using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents an action in the ROS graph.
/// </summary>
public class RosAction
{
    private readonly ConcurrentDictionary<RosActionEndPoint, RosActionEndPoint> _servers = new(), _clients = new();
    private readonly IEnumerator<KeyValuePair<RosActionEndPoint, RosActionEndPoint>>
        _serversEnumerator, _clientsEnumerator;

    internal RosAction(string name)
    {
        Name = name;
        _serversEnumerator = _servers.GetEnumerator();
        _clientsEnumerator = _clients.GetEnumerator();
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

    internal void ResetServers(ReadOnlySpan<RosActionEndPoint> servers)
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

        foreach (var ep in servers) _servers[ep] = ep;
    }

    internal void ResetClients(ReadOnlySpan<RosActionEndPoint> clients)
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

        foreach (var ep in clients) _clients[ep] = ep;
    }
}