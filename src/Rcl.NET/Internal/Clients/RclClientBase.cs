using Rcl.Interop;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Internal.Clients;
internal abstract class RclClientBase : RclWaitObject<SafeClientHandle>
{

    private readonly RclNodeImpl _node;
    private readonly Dictionary<long, ManualResetValueTaskSource<RosMessageBuffer>> _pendingRequests = new();
    private readonly CancellationTokenSource _shutdownSignal = new();

    public unsafe RclClientBase(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typeSupport,
        ClientOptions options)
        : base(node.Context, new(node.Handle, typeSupport, serviceName, options.Qos))
    {
        _node = node;

        Name = StringMarshal.CreatePooledString(rcl_client_get_service_name(Handle.Object))!;

        RegisterWaitHandle();
    }

    public unsafe bool IsServerAvailable
    {
        get
        {
            bool available;
            rcl_service_server_is_available(_node.Handle.Object, Handle.Object, &available);
            return available;
        }
    }

    public Task<bool> TryWaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => _node.Graph.TryWaitForServiceServerAsync(Name!, timeoutMilliseconds, cancellationToken);

    public Task<bool> TryWaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        => _node.Graph.TryWaitForServiceServerAsync(Name!, timeout, cancellationToken);

    public Task WaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => _node.Graph.WaitForServiceServerAsync(Name!, timeoutMilliseconds, cancellationToken);

    public Task WaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        => _node.Graph.WaitForServiceServerAsync(Name!, timeout, cancellationToken);

    public Task WaitForServerAsync(CancellationToken cancellationToken = default)
        => _node.Graph.WaitForServiceServerAsync(Name!, cancellationToken);

    public string Name { get; }

    public unsafe bool IsValid
         => rcl_client_is_valid(Handle.Object);

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        var responseBuffer = CreateResponseBuffer();
        var keepBuffer = false;

        try
        {
            if (rcl_ret_t.RCL_RET_OK ==
                rcl_take_response_with_info(
                    Handle.Object, &header, responseBuffer.Data.ToPointer())
            )
            {
                if (_pendingRequests.Remove(header.request_id.sequence_number, out var future))
                {
                    future.SetResult(responseBuffer);
                    keepBuffer = true;
                }
                // var future = _pendingRequests.GetOrAdd(
                //     header.request_id.sequence_number, static x =>
                //     {
                //         var mts = ObjectPool.Rent<ManualResetValueTaskSource<RosMessageBuffer>>();
                //         mts.OnFinally(x =>
                //         {
                //             var m = ((ManualResetValueTaskSource<RosMessageBuffer>)x!);
                //             m.Reset();
                //             ObjectPool.Return(m);
                //         }, mts);

                //         // Attach a tag so that we can check whether the ValueTaskSource
                //         // is taken or newly created here.
                //         mts.Tag = mts;
                //         return mts;
                //     });

                // if (future.Tag != future)
                // {
                //     _pendingRequests.Remove(header.request_id.sequence_number, out _);
                // }
            }
        }
        finally
        {
            if (!keepBuffer)
            {
                responseBuffer.Dispose();
            }
        }
    }

    protected abstract RosMessageBuffer CreateResponseBuffer();

    public async Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        // TODO: Maybe use private ObjectPools?
        var completion = ObjectPool.Rent<ManualResetValueTaskSource<RosMessageBuffer>>();
        var timeoutCts = ObjectPool.Rent<CancellationTokenSource>();
        var cancelAfterReg = timeoutCts.CancelAfter(timeoutMilliseconds, _node);

        // Yielding back to the event loop is required to avoid the situation that response 
        // has already been received at the point we add the ValueTaskSource into _pendingRequests,
        // causing the ValueTask never receive its corresponding response.
        //
        // If rcl_send_request allow us to indicate the sequence number, then we could have
        // the ValueTaskSource registered before calling rcl_send_request, then no yielding is
        // needed andthis method can just return a plain ValueTask to save some allocations.
        await _node.Context.YieldIfNotCurrent();

        long sequence;
        unsafe
        {
            RclException.ThrowIfNonSuccess(
                rcl_send_request(Handle.Object, request.Data.ToPointer(), &sequence));
        }

        // The request must be added to _pendingRequests before registering cancellation token callback.
        // Because if the cancellation token is already completed upon registration, the callback is
        // called in place, and if the request is not in _pendingRequests, the request can never complete.

        // Does not need locking or concurrent dictionary because we are on the event loop
        _pendingRequests[sequence] = completion;

        var cancelArgs = ObjectPool.Rent<CancellationArgs>()
            .Reset(sequence, this, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

        var outerReg = cancellationToken.Register(static s => ((CancellationArgs)s!).CancelWithOuterToken(), cancelArgs);
        var disposeReg = _shutdownSignal.Token.Register(static s => ((CancellationArgs)s!).CancelAsDisposed(), cancelArgs);
        var timeoutReg = timeoutCts.Token.Register(static s => ((CancellationArgs)s!).CancelAsTimedOut(), cancelArgs);

        var completionArgs =
            ObjectPool.Rent<CompletionArgs>()
            .Reset(sequence, this, completion, outerReg, disposeReg, timeoutReg, timeoutCts, cancelArgs, cancelAfterReg);

        completion.RunContinuationsAsynchronously = true;
        completion.OnFinally(state => ((CompletionArgs)state!).Return(), completionArgs);

        // var added = _pendingRequests.GetOrAdd(sequence, completion);
        // if (added != completion)
        // {
        //     Console.WriteLine("Request completed synchronously.");

        //     // This is supposed to be called by ValueTaskSource after the ValueTask completes.
        //     // But since we failed to register the completion here, which means we already received 
        //     // the response, so we call Return here to free up resources.
        //     _pendingRequests.Remove(sequence, out _);
        //     completionArgs.Return();
        // }

        return await new ValueTask<RosMessageBuffer>(completion, completion.Version).ConfigureAwait(false);
    }

    public Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, TimeSpan timeout, CancellationToken cancellationToken = default)
        => InvokeAsync(request, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, CancellationToken cancellationToken = default)
        => InvokeAsync(request, Timeout.Infinite, cancellationToken);

    public override void Dispose()
    {
        if (!_shutdownSignal.IsCancellationRequested)
        {
            _shutdownSignal.Cancel();
            _shutdownSignal.Dispose();
        }

        base.Dispose();
    }

    private class CancellationArgs
    {
        public long Sequence { get; private set; }

        public RclClientBase This { get; private set; } = null!;

        public TimeSpan Timeout { get; private set; }

        public CancellationToken OuterCancellation { get; private set; }

        public CancellationArgs Reset(
            long sequence,
            RclClientBase @this,
            TimeSpan timeout,
            CancellationToken outerCancellationToken)
        {
            Sequence = sequence;
            This = @this;
            Timeout = timeout;
            OuterCancellation = outerCancellationToken;
            return this;
        }

        public void CancelWithOuterToken()
        {
            if (This._pendingRequests.Remove(Sequence, out var ctx))
            {
                ctx.SetException(new OperationCanceledException(OuterCancellation));
            }
        }

        public void CancelAsDisposed()
        {
            if (This._pendingRequests.Remove(Sequence, out var ctx))
            {
                ctx.SetException(new ObjectDisposedException(This.GetType().Name));
            }
        }

        public void CancelAsTimedOut()
        {
            if (This._pendingRequests.Remove(Sequence, out var ctx))
            {
                ctx.SetException(new TimeoutException($"ROS service request timed out after {Timeout}."));
            }
        }

        public void Return()
        {
            Sequence = default;
            This = default!;
            Timeout = default;
            OuterCancellation = default;

            ObjectPool.Return(this);
        }
    }

    private class CompletionArgs
    {
        public long Sequence { get; private set; }

        public RclClientBase This { get; private set; } = null!;

        public ManualResetValueTaskSource<RosMessageBuffer> Completion { get; private set; } = null!;

        public CancellationTokenRegistration OuterCanellationReg { get; private set; }

        public CancellationTokenRegistration ShutdownCanellationReg { get; private set; }

        public CancellationTokenRegistration TimeoutCanellationReg { get; private set; }

        public TimeoutRegistration CancelAfterReg { get; private set; }

        public CancellationTokenSource TimeoutSource { get; private set; } = null!;

        public CancellationArgs CancellationArgs { get; private set; } = null!;

        public CompletionArgs Reset(
            long sequence,
            RclClientBase @this,
            ManualResetValueTaskSource<RosMessageBuffer> completion,
            CancellationTokenRegistration outerCancellation,
            CancellationTokenRegistration shutdownCancellation,
            CancellationTokenRegistration timeoutCancellation,
            CancellationTokenSource timeoutSource,
            CancellationArgs canelArgs,
            TimeoutRegistration cancelAfterReg)
        {
            Sequence = sequence;
            This = @this;
            Completion = completion;
            OuterCanellationReg = outerCancellation;
            ShutdownCanellationReg = shutdownCancellation;
            TimeoutCanellationReg = timeoutCancellation;
            TimeoutSource = timeoutSource;
            CancellationArgs = canelArgs;
            CancelAfterReg = cancelAfterReg;
            return this;
        }

        public void Return()
        {
            CancelAfterReg.Dispose();

            // Cancel CancellationTokenRegistrations
            OuterCanellationReg.Dispose();
            ShutdownCanellationReg.Dispose();
            TimeoutCanellationReg.Dispose();

            // Return TimeoutSource to the pool if not already canceled,
            // otherwise dispose it.
            if (TimeoutSource.TryReset())
            {
                ObjectPool.Return(TimeoutSource);
            }
            else
            {
                TimeoutSource.Dispose();
            }

            // Reset and return the ValueTaskSource
            Completion.Reset();
            ObjectPool.Return(Completion);

            // Reset and return the CancellationArgs
            CancellationArgs.Return();

            // Reset fields and return current instance to the pool.
            Sequence = default;
            @This = default!;
            Completion = default!;
            OuterCanellationReg = default;
            ShutdownCanellationReg = default;
            TimeoutCanellationReg = default;
            TimeoutSource = default!;
            CancellationArgs = default!;
            CancelAfterReg = default;
            ObjectPool.Return(this);
        }
    }
}