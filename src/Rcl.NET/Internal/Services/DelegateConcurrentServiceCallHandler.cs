using Rosidl.Runtime;

namespace Rcl.Internal.Services;

internal class DelegateConcurrentServiceCallHandler<TRequest, TResponse> : IConcurrentServiceHandler<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    private readonly object? _state;
    private readonly Func<TRequest, object?, CancellationToken, Task<TResponse>> _handler;

    public DelegateConcurrentServiceCallHandler(Func<TRequest, object?, CancellationToken, Task<TResponse>> handler, object? state)
    {
        _state = state;
        _handler = handler;
    }

    public Task<TResponse> ProcessRequestAsync(TRequest request, CancellationToken cancellationToken = default)
        => _handler(request, _state, cancellationToken);
}