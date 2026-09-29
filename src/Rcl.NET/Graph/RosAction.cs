using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents an action in the ROS graph.
/// </summary>
public class RosAction
{
    internal readonly SnapshotCollection<RosActionEndPoint> ServersSnapshot;
    internal readonly SnapshotCollection<RosActionEndPoint> ClientsSnapshot;

    private readonly ConcurrentDictionary<RosActionEndPoint, RosActionEndPoint> _servers = new(), _clients = new();

    internal RosAction(string name, SnapshotPublisher publisher)
    {
        ServersSnapshot = new(publisher);
        ClientsSnapshot = new(publisher);

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
    public IReadOnlyCollection<RosActionEndPoint> Servers => ServersSnapshot.GetSnapshot();

    /// <summary>
    /// Gets a list of available ROS action clients registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Clients => ClientsSnapshot.GetSnapshot();

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