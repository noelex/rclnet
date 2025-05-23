﻿using Rcl.Logging;
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
    private static int s_contextRefCount = 0;
    private static readonly ObjectPool<ManualResetValueTaskSource<bool>> s_tcsPool = ObjectPool<ManualResetValueTaskSource<bool>>.Shared;

    private readonly RclSynchronizationContext _rclSyncContext;

    private SpinLock _handleLock = new(), _callbackLock = new();
    private readonly Queue<CallbackWorkItem> _callbacks = new();
    private readonly Dictionary<long, WaitSetWorkItem> _waitHandles = new();
    private uint _cGuardConditions, _cTimers, _cEvents, _cSubscriptions, _cServices, _cClients;

    private readonly SafeGuardConditionHandle _interruptSignal, _shutdownSignal;
    private readonly SafeContextHandle _context;
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

        var allocator = RclAllocator.Default.Object;

        if (Interlocked.Increment(ref s_contextRefCount) == 1)
        {
            RclException.ThrowIfNonSuccess(
                rcl_logging_configure(&_context.Object->global_arguments, &allocator));
        }

        _loggerFactory = loggerFactory ?? new RcutilsLoggerFactory();
        DefaultLogger = CreateLogger("rclnet");

        _interruptSignal = new SafeGuardConditionHandle(_context);
        _shutdownSignal = new SafeGuardConditionHandle(_context);
        _useSyncContext = useSynchronizationContext;

        _mainLoopRunner = new(Run)
        {
            Name = "RCL Event Loop"
        };
        _mainLoopRunner.Start();
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

    private unsafe void Interrupt() => rcl_trigger_guard_condition(_interruptSignal.Object);

    private unsafe void DisposeCore(bool blocking)
    {
        if (Interlocked.CompareExchange(ref _disposed, 1, 0) == 0)
        {
            foreach (var feature in _features)
            {
                if (feature.Value is IDisposable d)
                {
                    d.Dispose();
                }
            }
            _features.Clear();

            rcl_trigger_guard_condition(_shutdownSignal.Object);

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
        if (Volatile.Read(ref _disposed) == 1) throw new ObjectDisposedException(nameof(RclContext));
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

    private void RegisterWaitHandle(long token, RclObjectHandle handle, Action<RclObjectHandle, object?> callback, object? state)
    {
        ThrowIfDisposed();

        if (handle.IsInvalid || handle.IsClosed)
        {
            var name = handle.GetType().Name;
            throw new ObjectDisposedException(name, $"Unable to register '{name}' as it's already disposed.");
        }

        using (ScopedLock.Lock(ref _handleLock))
        {
            _waitHandles[token] = new(handle, callback, state);
            switch (handle)
            {
                case SafeGuardConditionHandle:
                    _cGuardConditions++;
                    break;
                case SafeTimerHandle:
                    _cTimers++;
                    break;
                case SafeSubscriptionHandle:
                    _cSubscriptions++;
                    break;
                case SafeServiceHandle:
                    _cServices++;
                    break;
                case SafeClientHandle:
                    _cClients++;
                    break;
                case SafePublisherEventHandle:
                case SafeSubscriptionEventHandle:
                    _cEvents++;
                    break;
            }
        }

        Interrupt();
    }

    private void UnregisterWaitHandle(long token)
    {
        using (ScopedLock.Lock(ref _handleLock))
        {
            if (_waitHandles.Remove(token, out var v))
            {
                switch (v.WaitHandle)
                {
                    case SafeGuardConditionHandle:
                        _cGuardConditions--;
                        break;
                    case SafeTimerHandle:
                        _cTimers--;
                        break;
                    case SafeSubscriptionHandle:
                        _cSubscriptions--;
                        break;
                    case SafeServiceHandle:
                        _cServices--;
                        break;
                    case SafeClientHandle:
                        _cClients--;
                        break;
                    case SafePublisherEventHandle:
                    case SafeSubscriptionEventHandle:
                        _cEvents--;
                        break;
                }
            }
        }

        Interrupt();
    }

    internal WaitHandleRegistration Register(SafeTimerHandle handle, Action<RclObjectHandle, object?> callback, object? state = null)
    {
        var token = Interlocked.Increment(ref _waitHandleToken);
        RegisterWaitHandle(token, handle, callback, state);
        return new WaitHandleRegistration(this, static (ctx, x) => ctx.UnregisterWaitHandle(x), token);
    }

    private WaitHandleRegistration RegisterCore<T>(RclContextualObject<T> waitObject, Action<RclObjectHandle, object?> callback, object? state = null)
        where T : RclObjectHandle
    {
        var token = Interlocked.Increment(ref _waitHandleToken);
        RegisterWaitHandle(token, waitObject.Handle, callback, state);
        return new WaitHandleRegistration(this, static (ctx, x) => ctx.UnregisterWaitHandle(x), token);
    }

    internal WaitHandleRegistration Register<T>(RclWaitObject<T> guardCondition, Action<RclObjectHandle, object?> callback, object? state = null)
        where T : RclObjectHandle
            => RegisterCore(guardCondition, callback, state);

    private unsafe void Run()
    {
        if (_useSyncContext)
        {
            SynchronizationContext.SetSynchronizationContext(SynchronizationContext);
        }

        var ws = rcl_get_zero_initialized_wait_set();
        rcl_wait_set_init(&ws, 0, 0, 0, 0, 0, 0, _context.Object, RclAllocator.Default.Object);

        var callbacks = new List<CallbackWorkItem>();
        var waitHandles = new Dictionary<nint, WaitSetWorkItem>();

        bool isShutdownRequested = false;
        size_t idx;

        while (!isShutdownRequested)
        {
            try
            {
                using (ScopedLock.Lock(ref _handleLock))
                {
                    rcl_wait_set_resize(&ws,
                        _cSubscriptions,
                        _cGuardConditions + 2, // +2 For interrupt & shutdown guard conditions.
                        _cTimers,
                        _cClients,
                        _cServices,
                        _cEvents);

                    rcl_wait_set_add_guard_condition(&ws, _interruptSignal.Object, &idx);
                    rcl_wait_set_add_guard_condition(&ws, _shutdownSignal.Object, &idx);

                    foreach (var (key, value) in _waitHandles)
                    {
                        waitHandles.Add(value.WaitHandle.DangerousGetHandle(), value);

                        switch (value.WaitHandle)
                        {
                            case SafeGuardConditionHandle guardCondition:
                                rcl_wait_set_add_guard_condition(&ws, guardCondition.Object, &idx);
                                break;
                            case SafeTimerHandle timer:
                                rcl_wait_set_add_timer(&ws, timer.Object, &idx);
                                break;
                            case SafeSubscriptionHandle subscription:
                                rcl_wait_set_add_subscription(&ws, subscription.Object, &idx);
                                break;
                            case SafeServiceHandle service:
                                rcl_wait_set_add_service(&ws, service.Object, &idx);
                                break;
                            case SafeClientHandle client:
                                rcl_wait_set_add_client(&ws, client.Object, &idx);
                                break;
                            case SafePublisherEventHandle pubEvent:
                                rcl_wait_set_add_event(&ws, (rcl_event_t*)pubEvent.DangerousGetHandle().ToPointer(), &idx);
                                break;
                            case SafeSubscriptionEventHandle subEvent:
                                rcl_wait_set_add_event(&ws, (rcl_event_t*)subEvent.DangerousGetHandle().ToPointer(), &idx);
                                break;
                        }
                    }
                }

                RclException.ThrowIfNonSuccess(rcl_wait(&ws, -1));

                // The order of the following checks matters,
                // higher priority wait objects should be checked first.

                // Check for timers.
                for (uint i = 0; i < ws.size_of_timers; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws.timers[i]));
                }

                // Check for subscriptions.
                for (uint i = 0; i < ws.size_of_subscriptions; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws.subscriptions[i]));
                }

                // Check for incoming service calls.
                for (uint i = 0; i < ws.size_of_services; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws.services[i]));
                }

                // Check for outgoing service calls.
                for (uint i = 0; i < ws.size_of_clients; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws.clients[i]));
                }

                // Check for events.
                for (uint i = 0; i < ws.size_of_events; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws.events[i]));
                }

                // Check for guard conditions.
                // Skips interrupt & shutdown signal.
                for (uint i = 2; i < ws.size_of_guard_conditions; i++)
                {
                    CallIfCompleted(waitHandles, new nint(ws.guard_conditions[i]));
                }

                // Are we shutting down?
                if (_shutdownSignal.DangerousGetHandle() == new nint(ws.guard_conditions[1]))
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
                waitHandles.Clear();
                callbacks.Clear();
            }
        }

        rcl_wait_set_fini(&ws);
        _interruptSignal.Dispose();
        _shutdownSignal.Dispose();
        _context.Dispose();

        if (Interlocked.Decrement(ref s_contextRefCount) == 0)
        {
            rcl_logging_fini();
        }

        _shutdownComplete.SetResult();
    }

    private void CallIfCompleted(Dictionary<nint, WaitSetWorkItem> registry, nint completedHandle)
    {
        if (completedHandle == nint.Zero) return;

        try
        {
            var wh = registry[completedHandle];
            wh.Callback(wh.WaitHandle, wh.State);
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
        return (T)_features.GetOrAdd(name, featureFactory);
    }

    private record struct WaitSetWorkItem(RclObjectHandle WaitHandle, Action<RclObjectHandle, object?> Callback, object? State);

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
                _context.RegisterCallback(d, state, null);
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