using System.Collections.Concurrent;

namespace Rcl.Graph;

public enum ActionEndPointType
{
    Server,
    Client
}

public record RosActionEndPoint
    (
        RosNode Node,
        ActionEndPointType EndPointType,
        RosAction Action,
        string Type
    )
{
    public override string ToString()
    {
        return $"{Enum.GetName(EndPointType)}(Action = {Action}, Type = {Type})";
    }
}

/// <summary>
/// Represents an action in the ROS graph.
/// </summary>
public class RosAction
{
    private readonly ConcurrentBag<RosActionEndPoint> _servers = new(), _clients = new();

    public RosAction(string name)
    {
        Name = name;
        Servers = ReadOnlyCollection.Wrap(_servers);
        Clients = ReadOnlyCollection.Wrap(_clients);
    }

    /// <summary>
    /// Gets the name of the ROS action.
    /// </summary>
    public string Name { get; }

    public override string ToString()
    {
        return Name;
    }

    /// <summary>
    /// Gets a list of available ROS action servers registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Servers { get; }

    /// <summary>
    /// Gets a list of available ROS action clients registered with current <see cref="RosAction"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> Clients { get; }

    internal void ResetServers(IEnumerable<RosActionEndPoint> servers)
    {
        _servers.Clear();
        foreach (var ep in servers) _servers.Add(ep);
    }

    internal void ResetClients(IEnumerable<RosActionEndPoint> clients)
    {
        _clients.Clear();
        foreach (var ep in clients) _clients.Add(ep);
    }
}