using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a service in the ROS graph.
/// </summary>
public class RosService
{
    internal readonly SnapshotCollection<RosServiceEndPoint> ServersSnapshot;
    internal readonly SnapshotCollection<RosServiceEndPoint> ClientsSnapshot;

    private readonly ConcurrentDictionary<RosServiceEndPoint, RosServiceEndPoint> _servers = new(), _clients = new();

    internal RosService(string name, SnapshotPublisher publisher)
    {
        ServersSnapshot = new(publisher);
        ClientsSnapshot = new(publisher);

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
    public IReadOnlyCollection<RosServiceEndPoint> Servers => ServersSnapshot.GetSnapshot();

    /// <summary>
    /// Gets a list of available ROS service clients registered with current <see cref="RosService"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Clients => ClientsSnapshot.GetSnapshot();

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