using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents an action in the ROS graph.
/// </summary>
public class RosAction
{
    private readonly ConcurrentDictionary<RosActionEndPoint, RosActionEndPoint> _servers = new(), _clients = new();

    internal RosAction(string name)
    {
        Name = name;
    }

    /// <summary>
    /// Gets the name of the ROS action.
    /// </summary>
    public string Name { get; }

    /// <inheritdoc/>
    public override string ToString()
    {
        return Name;
    }

    /// <summary>
    /// Gets a list of available ROS action servers registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Servers => (IReadOnlyCollection<RosActionEndPoint>)_servers.Values;

    /// <summary>
    /// Gets a list of available ROS action clients registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Clients => (IReadOnlyCollection<RosActionEndPoint>)_clients.Values;

    internal int ServerCount => _servers.Count;

    internal int ClientCount => _clients.Count;

    internal void RemoveServer(RosActionEndPoint ep)
    {
        _servers.Remove(ep, out _);
    }

    internal void RemoveClient(RosActionEndPoint ep)
    {
        _clients.Remove(ep, out _);
    }

    internal void AddServer(RosActionEndPoint ep)
    {
        _servers[ep] = ep;
    }

    internal void AddClient(RosActionEndPoint ep)
    {
        _clients[ep] = ep;
    }
}