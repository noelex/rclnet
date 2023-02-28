using Rcl.Logging;
using Rcl.Logging.Impl;
using Rcl.SafeHandles;
using System.Collections.Concurrent;
using System.Diagnostics;

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
public sealed unsafe class RclContext : IDisposable, IRclContext
{
    private static int _contextRefCount = 0;

    private static readonly ObjectPool<ManualResetValueTaskSource<bool>> TcsPool = ObjectPool<ManualResetValueTaskSource<bool>>.Shared;

    private readonly SynchronizationContext _rclSyncContext;

    private SpinLock _handleLock = new(), _callbackLock = new();
    private readonly Dictionary<long, WaitSetWorkItem> _waitHandles = new();
    private readonly Queue<CallbackWorkItem> _callbacks = new();

    private readonly SafeGuardConditionHandle _interruptSignal, _shutdownSignal;
    private readonly SafeContextHandle _context;
    private readonly Thread _mainLoopRunner;

    private readonly ConcurrentDictionary<string, object> _features = new();

    private readonly IRclLoggerFactory _loggerFactory;

    private int _disposed;
    private long _waitHandleToken;

    /// <summary>
    /// Creates a new <see cref="RclContext"/> with specified arguments and logger factory.
    /// </summary>
    /// <param name="args">Command line arguments to be passed to the context.</param>
    /// <param name="loggerFactory">
    /// A custom <see cref="IRclLoggerFactory"/> for creating loggers in the <see cref="RclContext"/>.
    /// </param>
    public RclContext(string[] args, IRclLoggerFactory? loggerFactory = null)
    {
        if (!RosEnvironment.IsFoxy && !RosEnvironment.IsHumble)
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

        _rclSyncContext = new RclSynchronizationContext(this);
        _context = new SafeContextHandle(args);

        var allocator = RclAllocator.Default.Object;

        if (Interlocked.Increment(ref _contextRefCount) == 1)
        {
            RclException.ThrowIfNonSuccess(
                rcl_logging_configure(&_context.Object->global_arguments, &allocator));
        }

        _loggerFactory = loggerFactory ?? new RcutilsLoggerFactory(this);
        DefaultLogger = CreateLogger("rclnet");

        _interruptSignal = new SafeGuardConditionHandle(_context);
        _shutdownSignal = new SafeGuardConditionHandle(_context);

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
    public IRclTimer CreateTimer(RclClock clock, TimeSpan period) => new RclTimer(this, clock.Impl, period);

    /// <inheritdoc/>
    public IRclTimer CreateTimer(TimeSpan period) => CreateTimer(RclClock.Ros, period);

    /// <inheritdoc/>
    public IRclNode CreateNode(string name, string @namespace = "/", NodeOptions? options = null)
        => new RclNodeImpl(this, name, @namespace, options);

    /// <inheritdoc/>
    public YieldAwaiter Yield()
        => new(_rclSyncContext);

    private unsafe void Interrupt() => rcl_trigger_guard_condition(_interruptSignal.Object);

    /// <inheritdoc/>
    public void Dispose()
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
        }
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

    private void RegisterWaitHandle(long token, RclObjectHandle handle, Action<object?> callback, object? state)
    {
        ThrowIfDisposed();

        using (ScopedLock.Lock(ref _handleLock))
        {
            _waitHandles[token] = new(handle, callback, state);
        }

        Interrupt();
    }

    private void UnregisterWaitHandle(long token)
    {
        using (ScopedLock.Lock(ref _handleLock))
        {
            _waitHandles.Remove(token);
        }

        Interrupt();
    }

    internal WaitHandleRegistration Register(SafeTimerHandle handle, Action<object?> callback, object? state = null)
    {
        var token = Interlocked.Increment(ref _waitHandleToken);
        RegisterWaitHandle(token, handle, callback, state);
        return new WaitHandleRegistration(this, static (ctx, x) => ctx.UnregisterWaitHandle(x), token);
    }

    private WaitHandleRegistration RegisterCore<T>(RclContextualObject<T> waitObject, Action<object?> callback, object? state = null)
        where T : RclObjectHandle
    {
        var token = Interlocked.Increment(ref _waitHandleToken);
        RegisterWaitHandle(token, waitObject.Handle, callback, state);
        return new WaitHandleRegistration(this, static (ctx, x) => ctx.UnregisterWaitHandle(x), token);
    }

    internal WaitHandleRegistration Register<T>(RclWaitObject<T> guardCondition, Action<object?> callback, object? state = null)
        where T : RclObjectHandle
            => RegisterCore(guardCondition, callback, state);

    private unsafe void Run()
    {
        var ws = rcl_get_zero_initialized_wait_set();
        rcl_wait_set_init(&ws, 0, 0, 0, 0, 0, 0, _context.Object, RclAllocator.Default.Object);

        var waitHandles = new List<WaitSetWorkItem>();
        var callbacks = new List<CallbackWorkItem>();
        var completedHandles = new List<nint>();

        var subscriptions = new List<SafeSubscriptionHandle>();
        var guardConditions = new List<SafeGuardConditionHandle>();
        var timers = new List<SafeTimerHandle>();
        var clients = new List<SafeClientHandle>();
        var services = new List<SafeServiceHandle>();
        var events = new List<RclObjectHandle>();

        List<size_t> subscriptionIndices = new(),
        guardConditionIndices = new(),
        timerIndices = new(),
        clientsIndices = new(),
        servicesIndices = new(),
        eventIndices = new();

        var isShutdownRequested = false;

        while (!isShutdownRequested)
        {
            try
            {
                guardConditions.Add(_interruptSignal);
                guardConditions.Add(_shutdownSignal);

                using (ScopedLock.Lock(ref _handleLock))
                {
                    foreach (var (key, value) in _waitHandles)
                    {
                        waitHandles.Add(value);

                        switch (value.WaitHandle)
                        {
                            case SafeGuardConditionHandle guardCondition:
                                guardConditions.Add(guardCondition);
                                break;
                            case SafeTimerHandle timer:
                                timers.Add(timer);
                                break;
                            case SafeSubscriptionHandle sub:
                                subscriptions.Add(sub);
                                break;
                            case SafeServiceHandle server:
                                services.Add(server);
                                break;
                            case SafeClientHandle client:
                                clients.Add(client);
                                break;
                            case SafePublisherEventHandle pubEvent:
                                events.Add(pubEvent);
                                break;
                            case SafeSubscriptionEventHandle subEvent:
                                events.Add(subEvent);
                                break;
                        }
                    }
                }

                rcl_wait_set_resize(&ws,
                    (nuint)subscriptions.Count,
                    (nuint)guardConditions.Count,
                    (nuint)timers.Count,
                    (nuint)clients.Count,
                    (nuint)services.Count,
                    (nuint)events.Count);

                size_t idx;
                foreach (var subscription in subscriptions)
                {
                    rcl_wait_set_add_subscription(&ws, subscription.Object, &idx);
                    subscriptionIndices.Add(idx);
                }
                foreach (var guardCondition in guardConditions)
                {
                    rcl_wait_set_add_guard_condition(&ws, guardCondition.Object, &idx);
                    guardConditionIndices.Add(idx);
                }
                foreach (var timer in timers)
                {
                    rcl_wait_set_add_timer(&ws, timer.Object, &idx);
                    timerIndices.Add(idx);
                }
                foreach (var service in services)
                {
                    rcl_wait_set_add_service(&ws, service.Object, &idx);
                    servicesIndices.Add(idx);
                }
                foreach (var client in clients)
                {
                    rcl_wait_set_add_client(&ws, client.Object, &idx);
                    clientsIndices.Add(idx);
                }
                foreach (var @event in events)
                {
                    rcl_wait_set_add_event(&ws, (rcl_event_t*)@event.DangerousGetHandle().ToPointer(), &idx);
                    eventIndices.Add(idx);
                }

                RclException.ThrowIfNonSuccess(rcl_wait(&ws, -1));

                // The order of the following checks matters,
                // higher priority wait objects should be checked first.

                // Check for timers.
                for (var i = 0; i < timerIndices.Count; i++)
                {
                    idx = timerIndices[i];
                    var target = timers[i].DangerousGetHandle();
                    if (target == new nint(ws.timers[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

                // Check for subscriptions.
                for (var i = 0; i < subscriptionIndices.Count; i++)
                {
                    idx = subscriptionIndices[i];
                    var target = subscriptions[i].DangerousGetHandle();
                    if (target == new nint(ws.subscriptions[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

                // Check for service calls.
                for (var i = 0; i < servicesIndices.Count; i++)
                {
                    idx = servicesIndices[i];
                    var target = services[i].DangerousGetHandle();
                    if (target == new nint(ws.services[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

                // Check for service calls.
                for (var i = 0; i < clientsIndices.Count; i++)
                {
                    idx = clientsIndices[i];
                    var target = clients[i].DangerousGetHandle();
                    if (target == new nint(ws.clients[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

                // Check for events.
                for (var i = 0; i < eventIndices.Count; i++)
                {
                    idx = eventIndices[i];
                    var target = events[i].DangerousGetHandle();
                    if (target == new nint(ws.events[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

                // Check for guard conditions.
                // Skip interrupt signal @ 0, but check shutdown signal @ 1.
                for (var i = 1; i < guardConditionIndices.Count; i++)
                {
                    idx = guardConditionIndices[i];
                    var target = guardConditions[i].DangerousGetHandle();
                    if (target == new nint(ws.guard_conditions[idx]))
                    {
                        if (target == _shutdownSignal.DangerousGetHandle())
                        {
                            isShutdownRequested = true;
                        }
                        else
                        {
                            completedHandles.Add(target);
                        }
                    }
                }

                // Call registered callbacks for wait objects
                // in the order they were added into completedHandles.
                foreach (var completed in completedHandles)
                {
                    foreach (var wh in waitHandles)
                    {
                        // TODO: Handles may be diposed by previously invoked callbacks.
                        // Should we ignore disposed handles here?
                        // Or make sure handle disposable always happens asynchronously?
                        if (wh.WaitHandle.DangerousGetHandle() == completed)
                        {
                            try
                            {
                                wh.Callback(wh.State);
                            }
                            catch (Exception ex)
                            {
                                DefaultLogger.LogWarning($"Unhandled exception thrown by wait handle callback: {ex.Message}");
                                DefaultLogger.LogWarning(ex.StackTrace);
                            }
                            break;
                        }
                    }
                }

                // Invocation of callbacks MUST happen after the call to rcl_wait,
                // because callbacks may dispose wait object synchronously,
                // causing segmentation fault in rcl_wait.
                ExecuteCallbacks(callbacks);
            }
            finally
            {
                completedHandles.Clear();
                waitHandles.Clear();

                subscriptions.Clear();
                guardConditions.Clear();
                timers.Clear();
                clients.Clear();
                services.Clear();
                events.Clear();

                subscriptionIndices.Clear();
                guardConditionIndices.Clear();
                timerIndices.Clear();
                clientsIndices.Clear();
                servicesIndices.Clear();
                eventIndices.Clear();
            }
        }

        rcl_wait_set_fini(&ws);
        _interruptSignal.Dispose();
        _context.Dispose();

        if (Interlocked.Decrement(ref _contextRefCount) == 0)
        {
            rcl_logging_fini();
        }
    }

    private void ExecuteCallbacks(List<CallbackWorkItem> storage)
    {
        try
        {
            using (ScopedLock.Lock(ref _callbackLock))
            {
                while (_callbacks.TryDequeue(out var cb))
                {
                    storage.Add(cb);
                }
            }

            foreach (var cb in storage)
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
                        DefaultLogger.LogWarning("Unhandled exception thrown by scheduled callback: " + ex.Message);
                        DefaultLogger.LogWarning(ex.StackTrace);
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
            storage.Clear();
        }
    }

    internal T GetOrAddFeature<T>(string name, Func<string, T> featureFactory) where T : class
    {
        return (T)_features.GetOrAdd(name, featureFactory);
    }

    private record struct WaitSetWorkItem(RclObjectHandle WaitHandle, Action<object?> Callback, object? State);

    private record struct CallbackWorkItem(SendOrPostCallback Callback, object? State, ManualResetValueTaskSource<bool>? CompletionSource);

    private class RclSynchronizationContext : SynchronizationContext
    {
        private readonly RclContext _context;

        public RclSynchronizationContext(RclContext context)
        {
            _context = context;
        }

        public override void Post(SendOrPostCallback d, object? state)
        {
            _context.RegisterCallback(d, state, null);
        }

        public override void Send(SendOrPostCallback d, object? state)
        {
            SendAsync(d, state).AsTask().Wait();
        }

        private ValueTask SendAsync(SendOrPostCallback callback, object? state)
        {
            if (_context.IsCurrent)
            {
                callback(state);
                return ValueTask.CompletedTask;
            }

            var tcs = TcsPool.Rent();

            tcs.OnFinally(static state =>
            {
                var t = (ManualResetValueTaskSource<bool>)state!;
                t.Reset();
                TcsPool.Return(t);
            }, tcs);

            _context.RegisterCallback(callback, state, tcs);
            return new ValueTask(tcs, tcs.Version);
        }
    }
}