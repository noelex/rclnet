namespace Rcl.Internal.Services;

internal class DelegateConcurrentNativeServiceCallHandler : IConcurrentNativeServiceHandler
{
    private readonly object? _state;
    private readonly Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> _handler;

    public DelegateConcurrentNativeServiceCallHandler(Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> handler, object? state)
    {
        _state = state;
        _handler = handler;
    }


    public Task ProcessRequestAsync(RosMessageBuffer request, RosMessageBuffer response, CancellationToken cancellationToken = default)
        => _handler(request, response, _state, cancellationToken);
}