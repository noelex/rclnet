using Rcl.Logging;
using Rcl.Logging.Impl;
using Rcl.SafeHandles;
using System.Diagnostics.CodeAnalysis;

namespace Rcl;

/// <summary>
/// A context that, runs an event loop to provide asynchronous programming support for the rclnet library.
/// </summary>
/// <remarks>
/// <see cref="RclContext"/> servers as a host for other rcl concepts such as <see cref="IRclNode"/>s,
/// <see cref="IRclTimer"/>s and <see cref="IRclGuardCondition"/>s.
/// Applications can initiate as many <see cref="RclContext"/> as they want, but having a single context will
/// usually suffice.
/// <para>
/// Internally, <see cref="RclContext"/> runs an event loop to handle waitsets and callbacks, and provides
/// an asynchronous API for users to interact with these concepts easily.
/// </para>
/// </remarks>
public sealed unsafe class RclContext : IDisposable, IRclContext
{
    private static readonly ObjectPool<ManualResetValueTaskSource<bool>> TcsPool = ObjectPool<ManualResetValueTaskSource<bool>>.Shared;

    private readonly SynchronizationContext _rclSyncContext;

    private SpinLock _handleLock = new(), _callbackLock = new();
    private readonly Dictionary<long, WaitSetWorkItem> _waitHandles = new();
    private readonly Queue<CallbackWorkItem> _callbacks = new();

    private readonly SafeGuardConditionHandle _interruptSignal;
    private readonly SafeContextHandle _context;
    private readonly Thread _mainLoopRunner;

    private readonly Dictionary<string, object> _features = new();

    private int _disposed;
    private long _waitHandleToken;

    /// <summary>
    /// Initialize an <see cref="RclContext"/> and start the underlying event loop immediately.
    /// </summary>
    /// <param name="args">Command line arguments to be passed to the context.</param>
    public RclContext(string[] args, IRclLoggerFactory? loggerFactory = null)
    {
        if (!RosEnvironment.IsFoxy && !RosEnvironment.IsHumble)
        {
            throw new NotSupportedException($"ROS distribution '{RosEnvironment.Distribution}' is not supported.");
        }

        _rclSyncContext = new RclSynchronizationContext(this);
        _context = new SafeContextHandle(args);

        var allocator = RclAllocator.Default.Object;
        RclException.ThrowIfNonSuccess(
            rcl_logging_configure(&_context.Object->global_arguments, &allocator));

        LoggerFactory = loggerFactory ?? new RcutilsLoggerFactory(_rclSyncContext);
        DefaultLogger = LoggerFactory.CreateLogger("rclnet");

        _interruptSignal = new SafeGuardConditionHandle(_context);

        _mainLoopRunner = new(Run)
        {
            Name = "RCL Event Loop"
        };
        _mainLoopRunner.Start();
    }

    internal SafeContextHandle Handle => _context;

    internal IRclLoggerFactory LoggerFactory { get; }

    internal IRclLogger DefaultLogger { get; }

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
        foreach(var feature in _features)
        {
            if(feature.Value is IDisposable d)
            {
                d.Dispose();
            }
        }
        _features.Clear();

        if (Interlocked.CompareExchange(ref _disposed, 1, 0) == 0)
        {
            Interrupt();
        }
    }

    private void ThrowIfDisposed()
    {
        if (Volatile.Read(ref _disposed) == 1) throw new ObjectDisposedException(nameof(RclContext));
    }

    private void RegisterCallback(SendOrPostCallback callback, object? state, ManualResetValueTaskSource<bool>? completion)
    {
        ThrowIfDisposed();

        bool success = false;
        try
        {
            _callbackLock.Enter(ref success);
            _callbacks.Enqueue(new(callback, state, completion));
        }
        finally
        {
            if (success) _callbackLock.Exit();
        }

        Interrupt();
    }

    private void RegisterWaitHandle(long token, RclObjectHandle handle, Action<object?> callback, object? state)
    {
        ThrowIfDisposed();

        bool success = false;
        try
        {
            _handleLock.Enter(ref success);
            _waitHandles[token] = new(handle, callback, state);
        }
        finally
        {
            if (success) _handleLock.Exit();
        }

        Interrupt();
    }

    private void UnregisterWaitHandle(long token)
    {
        bool success = false;
        try
        {
            _handleLock.Enter(ref success);
            _waitHandles.Remove(token);
        }
        finally
        {
            if (success) _handleLock.Exit();
        }

        Interrupt();
    }

    internal WaitHandleRegistration Register(SafeTimerHandle handle, Action<object?> callback, object? state = null)
    {
        var token = Interlocked.Increment(ref _waitHandleToken);
        RegisterWaitHandle(token, handle, callback, state);
        return new WaitHandleRegistration(this, static (ctx, x) => ctx.UnregisterWaitHandle(x), token);
    }

    private WaitHandleRegistration RegisterCore<T>(RclObject<T> waitObject, Action<object?> callback, object? state = null)
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
        var events = new List<SafeGuardConditionHandle>();

        List<size_t> subscriptionIndices = new(),
        guardConditionIndices = new(),
        timerIndices = new(),
        clientsIndices = new(),
        servicesIndices = new(),
        eventsIndices = new();

        while (Volatile.Read(ref _disposed) == 0)
        {
            try
            {
                guardConditions.Add(_interruptSignal);

                bool success = false;
                try
                {
                    _handleLock.Enter(ref success);
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
                                // ...
                        }
                    }
                }
                finally
                {
                    if (success) _handleLock.Exit();
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
                    // ...
                }

                // TODO: Possible race condition
                //
                // Consider the following situation. Another thread queues a callback and trigger
                // interrupt signal immediately after ExecuteCallbacks(), but before rcl_wait start
                // waiting on the wait set.
                //
                // If there's no other wait handle in the wait set is triggered, rcl_wait will block
                // indefinitely, causing the registered callback never being executed.
                ExecuteCallbacks(callbacks);

                RclException.ThrowIfNonSuccess(rcl_wait(&ws, -1));

                // Check for guard conditions, skipping the first element
                // since it's the interrupt signal.
                for (var i = 1; i < guardConditionIndices.Count; i++)
                {
                    idx = guardConditionIndices[i];
                    var target = guardConditions[i].DangerousGetHandle();
                    if (target == new nint(ws.guard_conditions[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

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

                // Check for events here.

                // Call register callbacks for triggered signals.
                foreach (var wh in waitHandles)
                {
                    if (completedHandles.Contains(wh.WaitHandle.DangerousGetHandle()))
                    {
                        try
                        {
                            wh.Callback(wh.State);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"Unhandled exception thrown by wait handle callback: {ex.Message}");
                            Console.WriteLine(ex.StackTrace);
                        }
                    }
                }
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
                eventsIndices.Clear();
            }
        }

        // Ensure all callbacks are executed before we quit.
        ExecuteCallbacks(callbacks);

        rcl_wait_set_fini(&ws);
        _interruptSignal.Dispose();
        _context.Dispose();

        rcl_logging_fini();
    }

    private void ExecuteCallbacks(List<CallbackWorkItem> storage)
    {
        try
        {
            bool success = false;
            try
            {
                _callbackLock.Enter(ref success);
                while (_callbacks.TryDequeue(out var cb))
                {
                    storage.Add(cb);
                }
            }
            finally
            {
                if (success) _callbackLock.Exit();
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
                        Console.WriteLine("Unhandled exception thrown by scheduled callback: " + ex.Message);
                        Console.WriteLine(ex.StackTrace);
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

    internal void AddFeature<T>(string name, T feature)where T : class
    {
        _features.Add(name, feature);
    }

    internal T GetFeature<T>(string name) where T : class
    {
        return (T)_features[name];
    }

    internal bool TryGetFeature<T>(string name, [NotNullWhen(true)] out T? feature) where T : class
    {
        feature = null;
        if (_features.TryGetValue(name, out var v))
        {
            feature = (T)v;
            return true;
        }

        return false;
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
            if (Current == this)
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