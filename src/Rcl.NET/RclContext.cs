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
    private static readonly ObjectPool<ManualResetValueTaskSource<bool>> s_tcsPool = ObjectPool<ManualResetValueTaskSource<bool>>.Shared;

    private readonly RclSynchronizationContext _rclSyncContext;

    private SpinLock _callbackLock = new();

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
    {
        _rclSyncContext = new RclSynchronizationContext(this);
        _context = new SafeContextHandle(args);

        try
        {
            _loggerFactory = loggerFactory ?? new RcutilsLoggerFactory();
            DefaultLogger = CreateLogger("rclnet");
            _interruptSignal = new SafeGuardConditionHandle(_context);
            _shutdownSignal = new SafeGuardConditionHandle(_context);
            _waitSet = new SafeWaitSetHandle(_context);
            _useSyncContext = useSynchronizationContext;
            _mainLoopRunner = new(Run)
            {
                Name = "RCL Event Loop"
            };
            _mainLoopRunner.Start();
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

    private static unsafe void TriggerInfrastructureSignal(SafeGuardConditionHandle signal)
    {
        bool added = false;

        try
        {
            signal.DangerousAddRef(ref added);
            rcl_trigger_guard_condition(signal.DangerousObject);
        }
        catch (ObjectDisposedException)
        {
            // No wakeup is needed after the loop exits.
        }
        finally
        {
            if (added)
            {
                signal.DangerousRelease();
            }
        }
    }

    internal void NotifyTimerChanged() => Interrupt();

    private unsafe void DisposeCore(bool blocking)
    {
        bool close;
        KeyValuePair<string, object>[] features;

        lock (_context.LifecycleGate)
        {
            lock (RegistrationGate)
            {
                close = Interlocked.CompareExchange(ref _disposed, 1, 0) == 0;

                if (close)
                {
                    _context.TryBeginClose();
                    features = _features.ToArray();
                    _features.Clear();
                }
                else
                {
                    features = Array.Empty<KeyValuePair<string, object>>();
                }
            }
        }

        if (close)
        {
            StopRegistrations();

            foreach (var feature in features)
            {
                if (feature.Value is IDisposable d)
                {
                    d.Dispose();
                }
            }

            TriggerInfrastructureSignal(_shutdownSignal);

            if (blocking && !IsCurrent)
            {
                _mainLoopRunner.Join();
            }
        }
    }

    /// <summary>
    /// Prevents further jobs to be added into current <see cref="RclContext"/>, and signals the event loop to exit after finishing ongoing jobs.
    /// </summary>
    /// <remarks>
    /// When called from a thread other than the event loop of current <see cref="RclContext"/>, this method will block until the event loop is completely shutdown.
    /// Otherwise, this method is returned immediately.
    /// <para>
    /// To ensure shutdown of the event loop under all circumstances, use <see cref="DisposeAsync"/> instead.
    /// </para>
    /// </remarks>
    public void Dispose() => DisposeCore(true);

    /// <summary>
    /// Prevents further jobs to be added into current <see cref="RclContext"/>, and asynchronously wait until the event loop is shutdown.
    /// </summary>
    /// <returns></returns>
    public ValueTask DisposeAsync()
    {
        DisposeCore(false);
        return new ValueTask(_shutdownComplete.Task);
    }

    private void ThrowIfDisposed()
    {
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) == 1, typeof(RclContext));
    }

    private void RegisterCallback(SendOrPostCallback callback, object? state, ManualResetValueTaskSource<bool>? completion)
    {
        ThrowIfDisposed();

        using (ScopedLock.Lock(ref _callbackLock))
        {
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
                RunCleanup(entry.Closed, entry.State);
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
            RunCleanup(callback, state);
        }
    }

    internal static void RunCleanup(Action<object?> callback, object? state)
    {
        try
        {
            callback(state);
        }
        catch (Exception error)
        {
            try
            {
                HandleReleaseDiagnostics.Record(new("RclContext", "managed cleanup", null, error.ToString()));
            }
            catch
            {
                // Diagnostics cannot interrupt remaining cleanup.
            }
        }
    }

    internal static void DisposeResource(IDisposable? resource)
    {
        if (resource != null)
        {
            RunCleanup(static state => ((IDisposable)state!).Dispose(), resource);
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
                    }

                    return;
                }
            }

            if (entry != null)
            {
                if (entry.OnDetached != null)
                {
                    RunCleanup(entry.OnDetached, entry.State);
                }

                entry.WaitHandle.DangerousRelease();

                if (dependentCleanup != null)
                {
                    foreach (var item in dependentCleanup)
                    {
                        RunCleanup(item.Callback, item.State);
                    }
                }
            }
            else
            {
                RunCleanup(work.Callback!, work.State);
            }
        }
    }

    private unsafe void Run()
    {
        if (_useSyncContext)
        {
            SynchronizationContext.SetSynchronizationContext(SynchronizationContext);
        }

        // The runner exclusively owns this wait set until the finally below.
        var ws = _waitSet.DangerousObject;

        var callbacks = new List<CallbackWorkItem>();
        var waitHandles = new Dictionary<nint, WaitSetRegistration>();

        bool isShutdownRequested = false;
        size_t idx;

        try
        {
            while (!isShutdownRequested)
            {
                try
                {
                    lock (RegistrationGate)
                    {
                        rcl_wait_set_resize(ws,
                            _cSubscriptions,
                            _cGuardConditions + 2, // +2 For interrupt & shutdown guard conditions.
                            _cTimers,
                            _cClients,
                            _cServices,
                            _cEvents);

                        rcl_wait_set_add_guard_condition(ws, _interruptSignal.DangerousObject, &idx);
                        rcl_wait_set_add_guard_condition(ws, _shutdownSignal.DangerousObject, &idx);

                        foreach (var (key, value) in _waitHandles)
                        {
                            // Each entry owns a registration ref until both snapshots have been cleared.
                            waitHandles.Add(value.WaitHandle.DangerousGetHandle(), value);

                            switch (value.WaitHandle)
                            {
                                case SafeGuardConditionHandle guardCondition:
                                    rcl_wait_set_add_guard_condition(ws, guardCondition.DangerousObject, &idx);
                                    break;
                                case SafeTimerHandle timer:
                                    rcl_wait_set_add_timer(ws, timer.DangerousObject, &idx);
                                    break;
                                case SafeSubscriptionHandle subscription:
                                    rcl_wait_set_add_subscription(ws, subscription.DangerousObject, &idx);
                                    break;
                                case SafeServiceHandle service:
                                    rcl_wait_set_add_service(ws, service.DangerousObject, &idx);
                                    break;
                                case SafeClientHandle client:
                                    rcl_wait_set_add_client(ws, client.DangerousObject, &idx);
                                    break;
                                case SafePublisherEventHandle pubEvent:
                                    rcl_wait_set_add_event(ws, pubEvent.DangerousObject, &idx);
                                    break;
                                case SafeSubscriptionEventHandle subEvent:
                                    rcl_wait_set_add_event(ws, subEvent.DangerousObject, &idx);
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

                    // Are we shutting down?
                    if (_shutdownSignal.DangerousGetHandle() == new nint(ws->guard_conditions[1]))
                    {
                        // TODO: _shutdownSignal occasionally gets triggered unexpectedly
                        // when running with cyclonedds on Ubuntu.
                        // Make sure context disposal is actually requested before exiting
                        // the event loop.
                        if (Volatile.Read(ref _disposed) == 1)
                        {
                            isShutdownRequested = true;
                        }
                    }

                    // A cleanup callback may release native handles or callback buffers.
                    RclException.ThrowIfNonSuccess(rcl_wait_set_clear(ws));
                    waitHandles.Clear();
                    DrainCleanup();

                    // Snapshot callbacks.
                    using (ScopedLock.Lock(ref _callbackLock))
                    {
                        while (_callbacks.TryDequeue(out var cb))
                        {
                            callbacks.Add(cb);
                        }
                    }

                    // Invoke custom callbacks.
                    foreach (var cb in callbacks)
                    {
                        try
                        {
                            cb.Callback(cb.State);
                            cb.CompletionSource?.SetResult(true);
                        }
                        catch (Exception ex)
                        {
                            if (cb.CompletionSource is null)
                            {
                                DefaultLogger.LogFatal("Unhandled exception was thrown by a user callback: " + ex.Message);
                                DefaultLogger.LogFatal(ex.StackTrace);
                                throw;
                            }
                            else
                            {
                                cb.CompletionSource?.SetException(ex);
                            }
                        }
                    }
                }
                finally
                {
                    RclException.ThrowIfNonSuccess(rcl_wait_set_clear(ws));
                    waitHandles.Clear();
                    callbacks.Clear();
                    DrainCleanup();
                }
            }
        }
        finally
        {
            _waitSet.Dispose();
            waitHandles.Clear();
            StopRegistrations();
            DrainCleanup(stopping: true);
            _interruptSignal.Dispose();
            _shutdownSignal.Dispose();
            _context.Shutdown();
            _context.Dispose();
        }

        _shutdownComplete.TrySetResult();
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
            ex.ObjectName == wh.WaitHandle.GetType().Name &&
            (wh.WaitHandle.IsClosing || _context.IsClosing))
        {
            // Close can win admission after the snapshot was taken.
        }
        catch (Exception ex)
        {
            DefaultLogger.LogFatal($"Unhandled exception was thrown by wait handle callback: {ex.Message}");
            DefaultLogger.LogFatal(ex.StackTrace);
            throw;
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

    private record struct CallbackWorkItem(SendOrPostCallback Callback, object? State, ManualResetValueTaskSource<bool>? CompletionSource);

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

            var tcs = s_tcsPool.Rent();

            tcs.RunContinuationsAsynchronously = true;
            tcs.OnFinally(static state =>
            {
                var t = (ManualResetValueTaskSource<bool>)state!;
                t.Reset();
                s_tcsPool.Return(t);
            }, tcs);

            _context.RegisterCallback(callback, state, tcs);
            return new ValueTask(tcs, tcs.Version);
        }
    }
}
