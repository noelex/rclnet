namespace Rcl.Internal.Services;

internal class DelegateNativeServiceCallHandler : INativeServiceHandler
{
    private readonly object? _state;
    private readonly Action<RosMessageBuffer, RosMessageBuffer, object?> _handler;

    public DelegateNativeServiceCallHandler(Action<RosMessageBuffer, RosMessageBuffer, object?> handler, object? state)
    {
        _state = state;
        _handler = handler;
    }

    public void ProcessRequest(RosMessageBuffer request, RosMessageBuffer response)
    {
        _handler(request, response, _state);
    }
}