using Rcl.Logging;
using Rcl.Logging.Impl;
using Rcl.SafeHandles;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl;

/// <summary>
/// A context that runs an event loop to provide asynchronous programming support for the rclnet library.
/// </summary>
/// <remarks>
/// <see cref="RclContext"/> serves as a host for other rcl concepts such as <see cref="IRclNode"/>s,
/// <see cref="IRclTimer"/>s and <see cref="IRclGuardCondition"/>s.
/// Applications can initiate as many <see cref="RclContext"/> as they want, but having a single context will
/// usually suffice.
/// <para>
/// Please note that rcl logging configuration is application wide, thus logging is only configured once with the
/// arguments used for creating the first <see cref="RclContext"/> instance.
/// </para>
/// </remarks>
public sealed class RclContext : IRclContext
{
    private readonly RclSynchronizationContext _rclSyncContext;

    internal object RegistrationGate { get; } = new();

    private readonly Queue<WaitSetRegistration> _removedRegistrations = new();
    private readonly Dictionary<nint, WaitSetRegistration> _registeredAddresses = new();
    private readonly Queue<(Action<object?> Callback, object? State)> _cleanup = new();
    private CleanupState _cleanupState;

    private enum CleanupState
    {
        Running, Draining, Stopped
    }

    private readonly Queue<CallbackWorkItem> _callbacks = new();
    private readonly Dictionary<long, WaitSetRegistration> _waitHandles = new();
    private uint _cGuardConditions, _cTimers, _cEvents, _cSubscriptions, _cServices, _cClients;

    private readonly SafeGuardConditionHandle _interruptSignal, _shutdownSignal;
    private readonly SafeContextHandle _context;
    private readonly SafeWaitSetHandle _waitSet;
    private readonly Thread _mainLoopRunner;

    private readonly ConcurrentDictionary<string, object> _features = new();

    private readonly IRclLoggerFactory _loggerFactory;
    private readonly bool _useSyncContext;

    private readonly TaskCompletionSource _shutdownComplete = new(TaskCreationOptions.RunContinuationsAsynchronously);

    private int _activeWakeups;
    private Exception? _wakeupFailure;
    private int _disposed;
    private long _waitHandleToken;

    unsafe static RclContext()
    {
        if (!RosEnvironment.IsSupported(RosEnvironment.Distribution))
        {
            string message;

            if (RosEnvironment.Distribution == string.Empty)
            {
                message =
                    "No ROS distribution detected. This usually indicates that either ROS is not installed on the system yet, " +
                    "or you have not sourced the setup files of an installed ROS distribution.";
            }
            else
            {
                message = $"ROS distribution '{RosEnvironment.Distribution}' is not supported.";
            }

            throw new NotSupportedException(message);
        }

        var lib = NativeLibrary.Load("rcutils", System.Reflection.Assembly.GetExecutingAssembly(), null);
        var isInitialized = Unsafe.AsRef<bool>(NativeLibrary.GetExport(lib, "g_rcutils_logging_initialized").ToPointer());

        if (!isInitialized)
        {
            RclException.ThrowIfNonSuccess(rcutils_logging_initialize());
        }
    }

    /// <summary>
    /// Creates a new <see cref="RclContext"/> with specified arguments and logger factory.
    /// </summary>
    /// <param name="args">Command line arguments to be passed to the context.</param>
    /// <param name="loggerFactory">
    /// A custom <see cref="IRclLoggerFactory"/> for creating loggers in the <see cref="RclContext"/>.
    /// </param>
    /// <param name="useSynchronizationContext">
    /// Whether to setup <see cref="SynchronizationContext"/> on the event loop, causing asynchronous continuations
    /// to always resume on the event loop by default.
    /// </param>
    public unsafe RclContext(string[] args, IRclLoggerFactory? loggerFactory = null, bool useSynchronizationContext = false)
        : this(new SafeContextHandle(args), loggerFactory, useSynchronizationContext)
    {
    }

    internal RclContext(SafeContextHandle context, IRclLoggerFactory? loggerFactory = null,
        bool useSynchronizationContext = false, Func<SafeContextHandle, SafeWaitSetHandle>? createWaitSet = null,
        Action<Thread>? startThread = null)
    {
        _rclSyncContext = new RclSynchronizationContext(this);
        _context = context;

        try
        {
            _loggerFactory = loggerFactory ?? new RcutilsLoggerFactory();
            DefaultLogger = CreateLogger("rclnet");
            _interruptSignal = new SafeGuardConditionHandle(_context);
            _shutdownSignal = new SafeGuardConditionHandle(_context);
            _waitSet = createWaitSet == null ? new SafeWaitSetHandle(_context) : createWaitSet(_context);
            _useSyncContext = useSynchronizationContext;
            _mainLoopRunner = new(Run)
            {
                Name = "RCL Event Loop"
            };
            if (startThread == null)
            {
                _mainLoopRunner.Start();
            }
            else
            {
                startThread(_mainLoopRunner);
            }
        }
        catch
        {
            _waitSet?.Dispose();
            _shutdownSignal?.Dispose();
            _interruptSignal?.Dispose();
            _context.Dispose();
            throw;
        }
    }

    /// <summary>
    /// Create a new <see cref="RclContext"/> with specified logger factory.
    /// </summary>
    /// <param name="loggerFactory">
    /// A custom <see cref="IRclLoggerFactory"/> for creating loggers in the <see cref="RclContext"/>.
    /// </param>
    public RclContext(IRclLoggerFactory loggerFactory)
        : this(Array.Empty<string>(), loggerFactory)
    {

    }

    /// <summary>
    /// Create a new <see cref="RclContext"/> with specified synchronization mode.
    /// </summary>
    /// <param name="useSynchronizationContext">
    /// Whether to setup <see cref="SynchronizationContext"/> on the event loop, causing asynchronous continuations
    /// to always resume on the event loop by default.
    /// </param>
    public RclContext(bool useSynchronizationContext)
        : this(Array.Empty<string>(), useSynchronizationContext: useSynchronizationContext)
    {

    }

    /// <summary>
    /// Create a new <see cref="RclContext"/>.
    /// </summary>
    public RclContext()
        : this(Array.Empty<string>())
    {

    }

    internal SafeContextHandle Handle => _context;

    internal IRclLogger DefaultLogger { get; }

    /// <inheritdoc/>
    public IRclLogger CreateLogger(string loggerName)
        => _loggerFactory.CreateLogger(loggerName);

    /// <inheritdoc/>
    public SynchronizationContext SynchronizationContext => _rclSyncContext;

    /// <inheritdoc/>
    public bool IsCurrent => Thread.CurrentThread == _mainLoopRunner;

    /// <inheritdoc/>
    public IRclGuardCondition CreateGuardCondition() => new RclGuardConditionImpl(this);

    /// <inheritdoc/>
    public IRclTimer CreateTimer(IRclClock clock, TimeSpan period)
    {
        if (clock is not RclClock rclClock)
        {
            throw new NotSupportedException("CreateTimer supports only RclClock.");
        }

        return new RclTimer(this, rclClock.Impl, period);
    }

    /// <inheritdoc/>
    public IRclTimer CreateTimer(TimeSpan period) => CreateTimer(RclClock.SteadyClock, period);

    /// <inheritdoc/>
    public IRclNode CreateNode(string name, string @namespace = "/", NodeOptions? options = null)
        => new RclNodeImpl(this, name, @namespace, null, options);

    /// <inheritdoc/>
    public IRclNode CreateNode(string name, IRclClock clockOverride, string @namespace = "/", NodeOptions? options = null)
    {
        if (clockOverride is not RclClock clock)
        {
            throw new NotSupportedException("CreateNode supports only RclClock.");
        }

        return new RclNodeImpl(this, name, @namespace, clock, options);
    }

    /// <inheritdoc/>
    public YieldAwaiter Yield()
        => new(_rclSyncContext, false);

    /// <summary>
    /// Creates an awaitable that asynchronously yields to a background thread when awaited.
    /// </summary>
    /// <returns>
    /// A context that, when awaited, will asynchronously transition into a background thread at the
    /// time of the await.
    /// </returns>
    public static YieldAwaiter YieldBackground()
        => new(null, false);

    /// <summary>
    /// Creates an awaitable that, when awaited, yield back to current <see cref="RclContext"/>
    /// if not executing on event loop.
    /// </summary>
    /// <returns></returns>
    internal YieldAwaiter YieldIfNotCurrent()
    {
        if (IsCurrent)
        {
            // Suppress yielding if already on the event loop.
            return new(_rclSyncContext, true);
        }

        return Yield();
    }

    private void Interrupt() => TriggerInfrastructureSignal(_interruptSignal);

    private void TriggerInfrastructureSignal(SafeGuardConditionHandle signal)
    {
        var error = TryTriggerInfrastructureSignal(signal);

        if (error == null)
        {
            return;
        }

        CloseAdmission();
        var alternate = ReferenceEquals(signal, _shutdownSignal) ? _interruptSignal : _shutdownSignal;
        var alternateError = TryTriggerInfrastructureSignal(alternate);

        if (alternateError != null)
        {
            // Both wakeups failed. Report failure without freeing resources still owned by
            // a potentially blocked native wait. Its runner will clean up if it returns.
            _shutdownComplete.TrySetException(new AggregateException(error, alternateError));
        }
    }

    private unsafe Exception? TryTriggerInfrastructureSignal(SafeGuardConditionHandle signal)
    {
        bool added = false;

        try
        {
            lock (RegistrationGate)
            {
                if (_cleanupState == CleanupState.Stopped)
                {
                    return null;
                }

                signal.DangerousAddRef(ref added);
                _activeWakeups++;
            }

            RclException.ThrowIfNonSuccess(rcl_trigger_guard_condition(signal.DangerousObject));
            return null;
        }
        catch (Exception error)
        {
            Interlocked.CompareExchange(ref _wakeupFailure, error, null);
            return error;
        }
        finally
        {
            if (added)
            {
                signal.DangerousRelease();

                lock (RegistrationGate)
                {
                    _activeWakeups--;
                    Monitor.PulseAll(RegistrationGate);
                }
            }
        }
    }

    internal void NotifyTimerChanged() => Interrupt();

    private bool CloseAdmission()
    {
        lock (_context.LifecycleGate)
        {
            lock (RegistrationGate)
            {
                if (_disposed != 0)
                {
                    return false;
                }

                _context.TryBeginClose();
                Volatile.Write(ref _disposed, 1);
            }
        }

        return true;
    }

    private void BeginClose()
    {
        if (CloseAdmission())
        {
            TriggerInfrastructureSignal(_shutdownSignal);
        }
    }

    private void DisposeCore(bool blocking)
    {
        BeginClose();

        if (blocking && !IsCurrent)
        {
            _shutdownComplete.Task.GetAwaiter().GetResult();
        }
    }

    /// <summary>
    /// Prevents further jobs to be added into current <see cref="RclContext"/>, and signals the event loop to exit after its active callback returns.
    /// </summary>
    /// <remarks>
    /// When called from a thread other than the event loop of current <see cref="RclContext"/>, this method will block until the event loop is completely shutdown.
    /// Otherwise, this method is returned immediately.
    /// <para>
    /// Concurrent callers observe the same shutdown result. Accepted continuations that have not started
    /// are transferred to the thread pool and are not included in shutdown completion.
    /// Shutdown failures are reported to callers waiting outside the event loop.
    /// </para>
    /// </remarks>
    public void Dispose() => DisposeCore(true);

    /// <summary>
    /// Prevents further jobs to be added into current <see cref="RclContext"/>, and asynchronously wait until the event loop is shutdown.
    /// </summary>
    /// <returns>A shared shutdown completion that faults if event-loop or shutdown cleanup fails.</returns>
    /// <remarks>
    /// Completion does not require user-owned children to be disposed. Their native dependencies keep
    /// Context storage and logging alive until the final handle reference is returned.
    /// </remarks>
    public ValueTask DisposeAsync()
    {
        DisposeCore(false);
        return new ValueTask(_shutdownComplete.Task);
    }

    private void ThrowIfDisposed()
    {
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) == 1, typeof(RclContext));
    }

    private void RegisterCallback(SendOrPostCallback callback, object? state, PendingOperation<bool>? completion)
    {
        lock (RegistrationGate)
        {
            ThrowIfDisposed();
            _callbacks.Enqueue(new(callback, state, completion));
        }

        Interrupt();
    }

    internal void Register(RclObjectHandle handle, Action<RclObjectHandle, object?> callback,
        object? state, ref WaitHandleRegistration owner, Action<object?>? closed = null, Action<object?>? detached = null)
    {
        bool added = false;

        try
        {
            handle.DangerousAddRef(ref added);

            lock (RegistrationGate)
            {
                ThrowIfDisposed();
                ObjectDisposedException.ThrowIf(_cleanupState != CleanupState.Running, typeof(RclContext));
                handle.ThrowIfOperationClosed();

                if (!ReferenceEquals(handle.Context, _context))
                {
                    throw new InvalidOperationException("The wait handle belongs to another context.");
                }

                if (!owner.IsEmpty)
                {
                    throw new InvalidOperationException("Registration is already published.");
                }

                var address = handle.DangerousGetHandle();

                if (_registeredAddresses.ContainsKey(address))
                {
                    throw new InvalidOperationException("The native entity is already registered.");
                }

                var entry = new WaitSetRegistration(this, ++_waitHandleToken, handle, callback, state, closed, detached);
                _registeredAddresses.Add(address, entry);

                try
                {
                    _waitHandles.Add(entry.Token, entry);
                }
                catch
                {
                    _registeredAddresses.Remove(address);
                    throw;
                }

                CountHandle(handle, true);
                owner = new(entry);
                added = false;
            }
        }
        finally
        {
            if (added)
            {
                handle.DangerousRelease();
            }
        }

        Interrupt();
    }

    private void CountHandle(RclObjectHandle handle, bool adding)
    {
        switch (handle)
        {
            case SafeGuardConditionHandle:
                if (adding) _cGuardConditions++; else _cGuardConditions--;
                break;
            case SafeTimerHandle:
                if (adding) _cTimers++; else _cTimers--;
                break;
            case SafeSubscriptionHandle:
                if (adding) _cSubscriptions++; else _cSubscriptions--;
                break;
            case SafeServiceHandle:
                if (adding) _cServices++; else _cServices--;
                break;
            case SafeClientHandle:
                if (adding) _cClients++; else _cClients--;
                break;
            case SafePublisherEventHandle:
            case SafeSubscriptionEventHandle:
                if (adding) _cEvents++; else _cEvents--;
                break;
        }
    }

    internal void UnregisterWaitHandle(WaitSetRegistration entry)
    {
        lock (RegistrationGate)
        {
            RemoveRegistration(entry);
        }

        Interrupt();
    }

    private void RemoveRegistration(WaitSetRegistration entry)
    {
        if (entry.RemoveRequested)
        {
            return;
        }

        entry.RemoveRequested = true;
        _waitHandles.Remove(entry.Token);
        _registeredAddresses.Remove(entry.WaitHandle.DangerousGetHandle());
        CountHandle(entry.WaitHandle, false);
        _removedRegistrations.Enqueue(entry);
    }

    private void StopRegistrations()
    {
        WaitSetRegistration[] entries;

        lock (RegistrationGate)
        {
            if (_cleanupState != CleanupState.Stopped)
            {
                _cleanupState = CleanupState.Draining;
            }

            entries = _waitHandles.Values.ToArray();

            foreach (var entry in entries)
            {
                RemoveRegistration(entry);
            }
        }

        foreach (var entry in entries)
        {
            if (entry.Closed != null)
            {
                Cleanup.Run(entry.Closed, entry.State);
            }
        }
    }

    internal void AfterDetach(WaitHandleRegistration registration, Action<object?> callback, object? state)
    {
        lock (RegistrationGate)
        {
            if (registration.Entry is { Detached: false } entry)
            {
                (entry.Cleanup ??= new()).Add((callback, state));
                return;
            }
        }

        ScheduleCleanup(callback, state);
    }

    internal void ScheduleCleanup(Action<object?> callback, object? state)
    {
        bool queued;

        lock (RegistrationGate)
        {
            queued = _cleanupState != CleanupState.Stopped;

            if (queued)
            {
                _cleanup.Enqueue((callback, state));
            }
        }

        if (queued)
        {
            Interrupt();
        }
        else
        {
            Cleanup.Run(callback, state);
        }
    }

    // Called only after clearing both native and managed snapshots, or after wait-set fini.
    private void DrainCleanup(bool stopping = false)
    {
        while (true)
        {
            WaitSetRegistration? entry = null;
            (Action<object?> Callback, object? State) work = default;
            List<(Action<object?> Callback, object? State)>? dependentCleanup = null;

            lock (RegistrationGate)
            {
                if (_removedRegistrations.TryDequeue(out entry))
                {
                    entry.Detached = true;
                    dependentCleanup = entry.Cleanup;
                    entry.Cleanup = null;
                }
                else if (!_cleanup.TryDequeue(out work))
                {
                    if (stopping)
                    {
                        _cleanupState = CleanupState.Stopped;

                        // An admitted native trigger must report its result before we
                        // publish shutdown success or release infrastructure owners.
                        while (_activeWakeups != 0)
                        {
                            Monitor.Wait(RegistrationGate);
                        }
                    }

                    return;
                }
            }

            if (entry != null)
            {
                if (entry.OnDetached != null)
                {
                    Cleanup.Run(entry.OnDetached, entry.State);
                }

                entry.WaitHandle.DangerousRelease();

                if (dependentCleanup != null)
                {
                    foreach (var item in dependentCleanup)
                    {
                        Cleanup.Run(item.Callback, item.State);
                    }
                }
            }
            else
            {
                Cleanup.Run(work.Callback!, work.State);
            }
        }
    }

    private unsafe void Run()
    {
        // The runner exclusively owns this wait set until the finally below.
        var ws = _waitSet.DangerousObject;

        var callbacks = new Queue<CallbackWorkItem>();
        var waitHandles = new Dictionary<nint, WaitSetRegistration>();

        List<Exception>? errors = null;
        size_t idx;

        try
        {
            if (_useSyncContext)
            {
                SynchronizationContext.SetSynchronizationContext(SynchronizationContext);
            }

            while (Volatile.Read(ref _disposed) == 0)
            {
                lock (RegistrationGate)
                {
                    RclException.ThrowIfNonSuccess(rcl_wait_set_resize(ws,
                        _cSubscriptions,
                        _cGuardConditions + 2, // +2 For interrupt & shutdown guard conditions.
                        _cTimers,
                        _cClients,
                        _cServices,
                        _cEvents));

                    RclException.ThrowIfNonSuccess(rcl_wait_set_add_guard_condition(ws, _interruptSignal.DangerousObject, &idx));
                    RclException.ThrowIfNonSuccess(rcl_wait_set_add_guard_condition(ws, _shutdownSignal.DangerousObject, &idx));

                    foreach (var (key, value) in _waitHandles)
                    {
                        // Each entry owns a registration ref until both snapshots have been cleared.
                        waitHandles.Add(value.WaitHandle.DangerousGetHandle(), value);

                        switch (value.WaitHandle)
                        {
                            case SafeGuardConditionHandle guardCondition:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_guard_condition(ws, guardCondition.DangerousObject, &idx));
                                break;
                            case SafeTimerHandle timer:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_timer(ws, timer.DangerousObject, &idx));
                                break;
                            case SafeSubscriptionHandle subscription:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_subscription(ws, subscription.DangerousObject, &idx));
                                break;
                            case SafeServiceHandle service:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_service(ws, service.DangerousObject, &idx));
                                break;
                            case SafeClientHandle client:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_client(ws, client.DangerousObject, &idx));
                                break;
                            case SafePublisherEventHandle pubEvent:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_event(ws, pubEvent.DangerousObject, &idx));
                                break;
                            case SafeSubscriptionEventHandle subEvent:
                                RclException.ThrowIfNonSuccess(rcl_wait_set_add_event(ws, subEvent.DangerousObject, &idx));
                                break;
                        }
                    }
                }

                var waitResult = rcl_wait(ws, -1);

                // RCL can shorten an infinite wait to the next timer deadline and
                // return timeout before a timer is ready. Keep processing callbacks
                // and rebuilding the wait set instead of terminating the event loop.
                if (waitResult != Rcl.Interop.rcl_ret_t.RCL_RET_TIMEOUT)
                {
                    RclException.ThrowIfNonSuccess(waitResult);
                }

                // The order of the following checks matters,
                // higher priority wait objects should be checked first.

                // Check for timers.
                for (uint i = 0; i < ws->size_of_timers; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws->timers[i]));
                }

                // Check for subscriptions.
                for (uint i = 0; i < ws->size_of_subscriptions; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws->subscriptions[i]));
                }

                // Check for incoming service calls.
                for (uint i = 0; i < ws->size_of_services; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws->services[i]));
                }

                // Check for outgoing service calls.
                for (uint i = 0; i < ws->size_of_clients; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws->clients[i]));
                }

                // Check for events.
                for (uint i = 0; i < ws->size_of_events; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws->events[i]));
                }

                // Check for guard conditions.
                // Skips interrupt & shutdown signal.
                for (uint i = 2; i < ws->size_of_guard_conditions; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws->guard_conditions[i]));
                }

                // A cleanup callback may release native handles or callback buffers.
                RclException.ThrowIfNonSuccess(rcl_wait_set_clear(ws));
                waitHandles.Clear();
                DrainCleanup();

                // Admission and close share the registration gate. Only callbacks that
                // start before close run here; the rest retain thread-pool fallback semantics.
                lock (RegistrationGate)
                {
                    while (_callbacks.TryDequeue(out var cb))
                    {
                        callbacks.Enqueue(cb);
                    }
                }

                while (TryTakeCallback(callbacks, out var cb))
                {
                    InvokeCallback(cb);
                }
            }
        }
        catch (Exception error)
        {
            (errors ??= new()).Add(error);
        }
        finally
        {
            // Every cleanup step is independent, and completion is published even after a fault.
            void Attempt(Action cleanup)
            {
                try
                {
                    cleanup();
                }
                catch (Exception error)
                {
                    (errors ??= new()).Add(error);
                }
            }

            Attempt(BeginClose);
            KeyValuePair<string, object>[] features;

            lock (_context.LifecycleGate)
            {
                features = _features.ToArray();
                _features.Clear();
            }

            foreach (var feature in features)
            {
                if (feature.Value is IDisposable resource)
                {
                    Attempt(resource.Dispose);
                }
            }

            Attempt(StopRegistrations);
            Attempt(_waitSet.RequestRelease);
            waitHandles.Clear();
            Attempt(() => DrainCleanup(stopping: true));

            lock (RegistrationGate)
            {
                while (_callbacks.TryDequeue(out var callback))
                {
                    callbacks.Enqueue(callback);
                }
            }

            while (callbacks.TryDequeue(out var callback))
            {
                ThreadPool.QueueUserWorkItem(static work => InvokeCallback(work), callback, preferLocal: false);
            }

            Attempt(_interruptSignal.RequestRelease);
            Attempt(_shutdownSignal.RequestRelease);
            Attempt(() =>
            {
                if (!_context.Shutdown())
                {
                    (errors ??= new()).Add(new InvalidOperationException(
                        "rcl_shutdown failed. See handle release diagnostics for the native error."));
                }
            });
            Attempt(_context.RequestRelease);

            if (_wakeupFailure != null)
            {
                (errors ??= new()).Add(_wakeupFailure);
            }

            if (errors == null)
            {
                _shutdownComplete.TrySetResult();
            }
            else
            {
                _shutdownComplete.TrySetException(errors);
            }
        }
    }

    private bool TryTakeCallback(Queue<CallbackWorkItem> callbacks, out CallbackWorkItem callback)
    {
        lock (RegistrationGate)
        {
            callback = default;
            return _disposed == 0 && callbacks.TryDequeue(out callback);
        }
    }

    private static void InvokeCallback(CallbackWorkItem callback)
    {
        try
        {
            callback.Callback(callback.State);
            callback.CompletionSource?.Succeed(true);
        }
        catch (Exception error)
        {
            if (callback.CompletionSource == null)
            {
                throw;
            }

            callback.CompletionSource.Fail(error);
        }
    }

    private void CallIfCompleted(Dictionary<nint, WaitSetRegistration> registry, nint completedHandle)
    {
        if (completedHandle == nint.Zero)
        {
            return;
        }

        var wh = registry[completedHandle];

        try
        {
            lock (RegistrationGate)
            {
                if (wh.RemoveRequested || wh.WaitHandle.IsClosing || _context.IsClosing)
                {
                    return;
                }
            }

            wh.Callback(wh.WaitHandle, wh.State);
        }
        catch (ObjectDisposedException ex) when (
            (ex.ObjectName == wh.WaitHandle.GetType().Name || ex.ObjectName == wh.WaitHandle.GetType().FullName) &&
            (wh.WaitHandle.IsClosing || _context.IsClosing))
        {
            // Close can win admission after the snapshot was taken.
        }
    }

    internal T GetOrAddFeature<T>(string name, Func<string, T> featureFactory) where T : class
    {
        lock (_context.LifecycleGate)
        {
            _context.ThrowIfOperationClosed();
            return (T)_features.GetOrAdd(name, featureFactory);
        }
    }

    private record struct CallbackWorkItem(SendOrPostCallback Callback, object? State, PendingOperation<bool>? CompletionSource);

    private class RclSynchronizationContext : SynchronizationContext
    {
        private readonly RclContext _context;

        public RclSynchronizationContext(RclContext context)
        {
            _context = context;
        }

        public override SynchronizationContext CreateCopy()
        {
            return new RclSynchronizationContext(_context);
        }

        public override void Post(SendOrPostCallback d, object? state)
        {
            // Fallback to default sync context if the underlying RclContext is already disposed.
            if (Volatile.Read(ref _context._disposed) == 1)
            {
                base.Post(d, state);
            }
            else
            {
                try
                {
                    _context.RegisterCallback(d, state, null);
                }
                catch (ObjectDisposedException)
                {
                    base.Post(d, state);
                }
            }
        }

        public override void Send(SendOrPostCallback d, object? state)
        {
            if (Volatile.Read(ref _context._disposed) == 1)
            {
                base.Send(d, state);
            }
            else
            {
                SendAsync(d, state).AsTask().GetAwaiter().GetResult();
            }
        }

        public ValueTask SendAsync(SendOrPostCallback callback, object? state)
        {
            if (_context.IsCurrent)
            {
                callback(state);
                return ValueTask.CompletedTask;
            }

            var pending = new PendingOperation<bool>(true, static (operation, error) => operation.Fail(error));

            try
            {
                _context.RegisterCallback(callback, state, pending);
            }
            catch (ObjectDisposedException)
            {
                // Close won publication. Preserve Send's fallback without leaking a pooled source.
                InvokeCallback(new(callback, state, pending));
            }
            catch (Exception error)
            {
                pending.Fail(error);
            }
            finally
            {
                pending.FinishSetup();
            }

            return pending.VoidTask;
        }
    }
}
