using Rcl.Interop;
using Rosidl.Runtime;

namespace Rcl.Internal.Services;

/// <summary>
/// This class allows handling multiple concurrent service requests asynchronously.
/// </summary>
internal class ConcurrentIntrospectionService : IntrospectionServiceBase
{
    private readonly RclNodeImpl _node;
    private readonly IConcurrentNativeServiceHandler _handler;
    private readonly CancellationTokenSource _shutdownSignal = new();

    public unsafe ConcurrentIntrospectionService(
        RclNodeImpl node,
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        TypeSupportHandle typesupport,
        ServerOptions options)
        : base(node, serviceName, typesupport, options)
    {
        _node = node;
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override unsafe void DispatchRequest(
        RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t id)
    {
        _ = DispatchAsync(request, response, id);
    }

    private async Task DispatchAsync(RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t requestId)
    {
        using (request)
        using (response)
        {
            await _handler.ProcessRequestAsync(request, response, _shutdownSignal.Token).ConfigureAwait(false);

            // Reacquire the native handle after the asynchronous handler completes.

            var ret = SendResponse(requestId, response.Data);

            if (ret != rcl_ret_t.RCL_RET_OK)
            {
                // Yield here since we want to propagate the error to the event loop.
                await _node.Context.YieldIfNotCurrent();
                RclException.ThrowIfNonSuccess(ret);
            }
        }
    }

    protected override void DisposeCore()
    {
        if (!_shutdownSignal.IsCancellationRequested)
        {
            _shutdownSignal.Cancel();
            _shutdownSignal.Dispose();
        }

        base.DisposeCore();
    }
}
