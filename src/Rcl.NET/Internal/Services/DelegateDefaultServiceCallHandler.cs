using Rosidl.Runtime;

namespace Rcl.Internal.Services;

internal class DelegateDefaultServiceCallHandler<TRequest, TResponse> : IServiceHandler<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    private readonly object? _state;
    private readonly Func<TRequest, object?, TResponse> _handler;

    public DelegateDefaultServiceCallHandler(Func<TRequest, object?, TResponse> handler, object? state)
    {
        _state = state;
        _handler = handler;
    }

    public TResponse ProcessRequest(TRequest request)
    {
        return _handler(request, _state);
    }
}
