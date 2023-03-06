﻿using Rcl.Logging;
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
public sealed class RclContext :  IRclContext
{
    private static int _contextRefCount = 0;

    // Creating nodes with /rosout logging enabled requires access
    // to a static logger map, which is not thread-safe application wide.
    //
    // Creation of other rcl objects is also not thread-safe, but limited to
    // the scope of a specific RclContext or RclNode, which can be resolved
    // by yielding to the owner RclContext before calling.
    //
    // Thus we only need a lock for node creation here.
    private static SpinLock  _nodeCreationLock = new();

    private static readonly ObjectPool<ManualResetValueTaskSource<bool>> TcsPool = ObjectPool<ManualResetValueTaskSource<bool>>.Shared;

    private readonly RclSynchronizationContext _rclSyncContext;

    private SpinLock _handleLock = new(), _callbackLock = new();
    private readonly Dictionary<long, WaitSetWorkItem> _waitHandles = new();
    private readonly Queue<CallbackWorkItem> _callbacks = new();

    private readonly SafeGuardConditionHandle _interruptSignal, _shutdownSignal;
    private readonly SafeContextHandle _context;
    private readonly Thread _mainLoopRunner;

    private readonly ConcurrentDictionary<string, object> _features = new();

    private readonly IRclLoggerFactory _loggerFactory;
    private readonly bool _useSyncContext;

    private readonly TaskCompletionSource _shutdownComplete = new(TaskCreationOptions.RunContinuationsAsynchronously);

    private int _disposed;
    private long _waitHandleToken;

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
    public IRclTimer CreateTimer(RclClock clock, TimeSpan period) => new RclTimer(this, clock.Impl, period);

    /// <inheritdoc/>
    public IRclTimer CreateTimer(TimeSpan period) => CreateTimer(RclClock.Steady, period);

    /// <inheritdoc/>
    public IRclNode CreateNode(string name, string @namespace = "/", NodeOptions? options = null)
    {
        using (ScopedLock.Lock(ref _nodeCreationLock))
        {
            return new RclNodeImpl(this, name, @namespace, options);
        }   
    }

    /// <inheritdoc/>
    public YieldAwaiter Yield()
        => new(_rclSyncContext, false);

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

    private void RegisterWaitHandle(long token, RclObjectHandle handle, Action<object?> callback, object? state)
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
        if (_useSyncContext)
        {
            SynchronizationContext.SetSynchronizationContext(SynchronizationContext);
        }

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

                // Invocation of callbacks MUST happen after the call to rcl_wait,
                // because callbacks may dispose wait object synchronously,
                // causing segmentation fault in rcl_wait.

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

                // Check for incoming service calls.
                for (var i = 0; i < servicesIndices.Count; i++)
                {
                    idx = servicesIndices[i];
                    var target = services[i].DangerousGetHandle();
                    if (target == new nint(ws.services[idx]))
                    {
                        completedHandles.Add(target);
                    }
                }

                // Check for outgoing service calls.
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
                        // TODO: Handles may have been diposed by previously invoked callbacks.
                        // Should we ignore disposed handles here?
                        // Or make sure handle disposal always happens asynchronously?
                        if (wh.WaitHandle.DangerousGetHandle() == completed)
                        {
                            try
                            {
                                wh.Callback(wh.State);
                            }
                            catch (Exception ex)
                            {
                                DefaultLogger.LogWarning($"Unhandled exception was thrown by wait handle callback: {ex.Message}");
                                DefaultLogger.LogWarning(ex.StackTrace);
                            }
                            break;
                        }
                    }
                }

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
        _shutdownSignal.Dispose();
        _context.Dispose();

        if (Interlocked.Decrement(ref _contextRefCount) == 0)
        {
            rcl_logging_fini();
        }

        _shutdownComplete.SetResult();
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
                        DefaultLogger.LogWarning("Unhandled exception was thrown by scheduled callback: " + ex.Message);
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
                SendAsync(d, state).AsTask().Wait();
            }
        }

        public ValueTask SendAsync(SendOrPostCallback callback, object? state)
        {
            if (_context.IsCurrent)
            {
                callback(state);
                return ValueTask.CompletedTask;
            }

            var tcs = TcsPool.Rent();

            tcs.RunContinuationsAsynchronously = true;
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