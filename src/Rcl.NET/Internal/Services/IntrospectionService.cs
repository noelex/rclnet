using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl.Internal.Services;

internal class IntrospectionService : IntrospectionServiceBase
{
    private readonly INativeServiceHandler _handler;

    private readonly RosMessageBuffer _requestBuffer, _responseBuffer;

    public unsafe IntrospectionService(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typesupport,
        INativeServiceHandler handler,
        QosProfile qos)
        : base(node, serviceName, typesupport, qos)
    {
        _handler = handler;

        _requestBuffer = base.CreateRequestBuffer();
        _responseBuffer = base.CreateResponseBuffer();
    }

    protected override RosMessageBuffer CreateRequestBuffer()
        => _requestBuffer;

    protected override RosMessageBuffer CreateResponseBuffer()
        => _responseBuffer;

    public override void Dispose()
    {
        if (!IsDisposed)
        {
            _requestBuffer.Dispose();
            _responseBuffer.Dispose();
        }

        base.Dispose();
    }

    protected override void OnTakeRequestFailed(RosMessageBuffer requestBuffer)
    {
        
    }

    protected unsafe override void DispatchRequest(
        RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t id)
    {
        _handler.ProcessRequest(request, response);

        RclException.ThrowIfNonSuccess(
            rcl_send_response(Handle.Object, &id, response.Data.ToPointer()));
    }
}