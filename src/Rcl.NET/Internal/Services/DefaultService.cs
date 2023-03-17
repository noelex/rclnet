using Rosidl.Runtime;
using System.Text;

namespace Rcl.Internal.Services;

/// <summary>
/// This class handles requests sequentialy on current <see cref="RclContext"/> event loop.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class DefaultService<TService, TRequest, TResponse>
    : IntrospectionService
    where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    public unsafe DefaultService(
        RclNodeImpl node,
        string serviceName,
        IServiceHandler<TRequest, TResponse> handler,
        ServerOptions options)
        : base(node, serviceName, TService.GetTypeSupportHandle(), new Handler(handler, options.TextEncoding), options)
    {
    }

    private class Handler : INativeServiceHandler
    {
        private readonly IServiceHandler<TRequest, TResponse> _handler;
        private readonly Encoding _textEncoding;

        public Handler(IServiceHandler<TRequest, TResponse> handler, Encoding textEncoding)
        {
            _handler = handler;
            _textEncoding = textEncoding;
        }

        public void ProcessRequest(RosMessageBuffer request, RosMessageBuffer response)
        {
            var req = (TRequest)TRequest.CreateFrom(request.Data, _textEncoding);
            var resp = _handler.ProcessRequest(req);
            resp.WriteTo(response.Data, _textEncoding);
        }
    }
}
