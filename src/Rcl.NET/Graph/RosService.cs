using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a service in the ROS graph.
/// </summary>
public class RosService
{
    private readonly ConcurrentDictionary<RosServiceEndPoint, RosServiceEndPoint> _servers = new(), _clients = new();
    private readonly IEnumerator<KeyValuePair<RosServiceEndPoint, RosServiceEndPoint>> _serversEnumerator, _clientsEnumerator;

    internal RosService(string name)
    {
        Name = name;
        _serversEnumerator = _servers.GetEnumerator();
        _clientsEnumerator = _clients.GetEnumerator();
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
    public IReadOnlyCollection<RosServiceEndPoint> Servers => (IReadOnlyCollection<RosServiceEndPoint>)_servers.Values;

    /// <summary>
    /// Gets a list of available ROS service clients registered with current <see cref="RosService"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Clients => (IReadOnlyCollection<RosServiceEndPoint>)_clients.Values;

    internal int ServerCount => _servers.Count;

    internal int ClientCount => _clients.Count;

    internal void ResetServers(Span<RosServiceEndPoint> servers)
    {
        try
        {
            while (_serversEnumerator.MoveNext())
            {
                var k = _serversEnumerator.Current.Key;
                if (!servers.Contains(k))
                {
                    _servers.Remove(k, out _);
                }
            }
        }
        finally
        {
            _serversEnumerator.Reset();
        }

        foreach (var item in servers) _servers[item] = item;
    }

    internal void ResetClients(Span<RosServiceEndPoint> clients)
    {
        try
        {
            while (_clientsEnumerator.MoveNext())
            {
                var k = _clientsEnumerator.Current.Key;
                if (!clients.Contains(k))
                {
                    _clients.Remove(k, out _);
                }
            }
        }
        finally
        {
            _clientsEnumerator.Reset();
        }

        foreach (var item in clients) _clients[item] = item;
    }
}