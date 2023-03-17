using Rosidl.Runtime;

namespace Rcl.Internal.Services;

internal class IntrospectionService : IntrospectionServiceBase
{
    private readonly INativeServiceHandler _handler;

    public unsafe IntrospectionService(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typesupport,
        INativeServiceHandler handler,
        ServerOptions options)
        : base(node, serviceName, typesupport, options)
    {
        _handler = handler;
    }

    protected unsafe override void DispatchRequest(
        RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t id)
    {
        // TODO: Reuse buffers by finializing rather than detroying after use.
        using (request)
        using (response)
        {
            _handler.ProcessRequest(request, response);

            RclException.ThrowIfNonSuccess(
                rcl_send_response(Handle.Object, &id, response.Data.ToPointer()));
        }
    }
}