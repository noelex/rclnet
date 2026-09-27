namespace Rcl.Graph;

/// <summary>
/// Delegate for handling <see cref="RosGraphEvent"/>s.
/// </summary>
/// <param name="args"></param>
public delegate void GraphChangedEventHandler(RosGraphEvent args);

public partial class RosGraph
{
    /// <summary>
    /// Listen to the event which will be triggered when ROS graph is changed.
    /// </summary>
    public event GraphChangedEventHandler? GraphChanged
    {
        add
        {
            lock (_handlersGate)
            {
                _handlers += value;
                Volatile.Write(ref _handlerSnapshot, _handlers?.GetInvocationList() ?? Array.Empty<Delegate>());
            }
        }
        remove
        {
            lock (_handlersGate)
            {
                _handlers -= value;
                Volatile.Write(ref _handlerSnapshot, _handlers?.GetInvocationList() ?? Array.Empty<Delegate>());
            }
        }
    }

    private readonly object _handlersGate = new();
    private GraphChangedEventHandler? _handlers;
    private Delegate[] _handlerSnapshot = Array.Empty<Delegate>();
}