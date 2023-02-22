using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a service in the ROS graph.
/// </summary>
public class RosService
{
    private readonly ConcurrentBag<RosServiceEndPoint> _servers = new(), _clients = new();

    public RosService(string name)
    {
        Name = name;
    }

    /// <summary>
    /// Gets the name of the ROS service.
    /// </summary>
    public string Name { get; }

    public override string ToString()
    {
        return Name;
    }

    /// <summary>
    /// Gets a list of available ROS service servers registered with current <see cref="RosService"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Servers => _servers;

    /// <summary>
    /// Gets a list of available ROS service clients registered with current <see cref="RosService"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Clients => _clients;

    internal void ResetServers(IEnumerable<RosServiceEndPoint> servers)
    {
        _servers.Clear();
        foreach (var item in servers) _servers.Add(item);
    }

    internal void ResetClients(IEnumerable<RosServiceEndPoint> clients)
    {
        _clients.Clear();
        foreach (var item in clients) _clients.Add(item);
    }
}