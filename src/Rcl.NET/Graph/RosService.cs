using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a service in the ROS graph.
/// </summary>
public class RosService
{
    private IReadOnlyCollection<RosServiceEndPoint> _serversSnapshot = Array.Empty<RosServiceEndPoint>();
    private IReadOnlyCollection<RosServiceEndPoint> _clientsSnapshot = Array.Empty<RosServiceEndPoint>();

    private readonly ConcurrentDictionary<RosServiceEndPoint, RosServiceEndPoint> _servers = new(), _clients = new();

    internal void PublishSnapshots(SnapshotChanges changes)
    {
        if ((changes & SnapshotChanges.Servers) != 0)
        {
            Volatile.Write(ref _serversSnapshot, (IReadOnlyCollection<RosServiceEndPoint>)_servers.Values);
        }

        if ((changes & SnapshotChanges.Clients) != 0)
        {
            Volatile.Write(ref _clientsSnapshot, (IReadOnlyCollection<RosServiceEndPoint>)_clients.Values);
        }
    }

    internal RosService(string name)
    {
        Name = name;
    }

    /// <summary>
    /// Gets the name of the ROS service.
    /// </summary>
    public string Name { get; }

    /// <inheritdoc/>
    public override string ToString()
    {
        return Name;
    }

    /// <summary>
    /// Gets a list of available ROS service servers registered with current <see cref="RosService"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Servers => Volatile.Read(ref _serversSnapshot);

    /// <summary>
    /// Gets a list of available ROS service clients registered with current <see cref="RosService"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Clients => Volatile.Read(ref _clientsSnapshot);

    internal int ServerCount => _servers.Count;

    internal int ClientCount => _clients.Count;

    internal void AddServer(RosServiceEndPoint server)
    {
        _servers[server] = server;
    }

    internal void AddClient(RosServiceEndPoint client)
    {
        _clients[client] = client;
    }

    internal void RemoveServer(RosServiceEndPoint server)
    {
        _servers.Remove(server, out _);
    }

    internal void RemoveClient(RosServiceEndPoint client)
    {
        _clients.Remove(client, out _);
    }
}