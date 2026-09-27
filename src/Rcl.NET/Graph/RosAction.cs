using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents an action in the ROS graph.
/// </summary>
public class RosAction
{
    private IReadOnlyCollection<RosActionEndPoint> _serversSnapshot = Array.Empty<RosActionEndPoint>();
    private IReadOnlyCollection<RosActionEndPoint> _clientsSnapshot = Array.Empty<RosActionEndPoint>();

    private readonly ConcurrentDictionary<RosActionEndPoint, RosActionEndPoint> _servers = new(), _clients = new();

    internal void PublishSnapshots(SnapshotChanges changes)
    {
        if ((changes & SnapshotChanges.Servers) != 0)
        {
            Volatile.Write(ref _serversSnapshot, (IReadOnlyCollection<RosActionEndPoint>)_servers.Values);
        }

        if ((changes & SnapshotChanges.Clients) != 0)
        {
            Volatile.Write(ref _clientsSnapshot, (IReadOnlyCollection<RosActionEndPoint>)_clients.Values);
        }
    }

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
    public IReadOnlyCollection<RosActionEndPoint> Servers => Volatile.Read(ref _serversSnapshot);

    /// <summary>
    /// Gets a list of available ROS action clients registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Clients => Volatile.Read(ref _clientsSnapshot);

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