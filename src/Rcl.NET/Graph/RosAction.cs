using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents an action in the ROS graph.
/// </summary>
public class RosAction
{
    private readonly ConcurrentBag<RosActionEndPoint> _servers = new(), _clients = new();

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
    public IReadOnlyCollection<RosActionEndPoint> Servers => _servers;

    /// <summary>
    /// Gets a list of available ROS action clients registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Clients => _clients;

    internal void ResetServers(ReadOnlySpan<RosActionEndPoint> servers)
    {
        _servers.Clear();
        foreach (var ep in servers) _servers.Add(ep);
    }

    internal void ResetClients(ReadOnlySpan<RosActionEndPoint> clients)
    {
        _clients.Clear();
        foreach (var ep in clients) _clients.Add(ep);
    }
}