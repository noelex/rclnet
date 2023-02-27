using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a node in the ROS graph.
/// </summary>
public class RosNode
{
    private readonly ConcurrentDictionary<NameWithType, RosServiceEndPoint>
        _servers = new(), _clients = new();

    private readonly ConcurrentBag<RosTopicEndPoint>
        _subscribers = new(), _publishers = new();

    private readonly ConcurrentDictionary<NameWithType, RosActionEndPoint>
        _actionServers = new(), _actionClients = new();

    internal RosNode(NodeName name, string enclave)
    {
        Name = name;
        Enclave = enclave;
    }

    /// <inheritdoc/>
    public override string ToString()
    {
        return Name.ToString();
    }

    /// <summary>
    /// Gets the fully-qualified name of the node.
    /// </summary>
    public NodeName Name { get; private set; }

    /// <summary>
    /// Gets the enclave name of the node.
    /// </summary>
    public string Enclave { get; private set; }

    /// <summary>
    /// Gets a list of <see cref="RosService"/> servers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Servers => (IReadOnlyCollection<RosServiceEndPoint>)_servers.Values;

    /// <summary>
    /// Gets a list of <see cref="RosService"/> clients registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Clients => (IReadOnlyCollection<RosServiceEndPoint>)_clients.Values;

    /// <summary>
    /// Gets a list of <see cref="RosTopic"/> subscribers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Subscribers => _subscribers;

    /// <summary>
    /// Gets a list of <see cref="RosTopic"/> publishers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Publishers => _publishers;

    /// <summary>
    /// Gets a list of <see cref="RosAction"/> servers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> ActionServers => (IReadOnlyCollection<RosActionEndPoint>)_actionServers.Values;

    /// <summary>
    /// Gets a list of <see cref="RosAction"/> clients registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> ActionClients => (IReadOnlyCollection<RosActionEndPoint>)_actionClients.Values;

    internal void UpdateServers(
        IGraphBuilder builder,
        ReadOnlySpan<NameWithType> discovered)
        => UpdateServiceEndPoints(builder, ServiceEndPointType.Server, discovered);

    internal void UpdateClients(
        IGraphBuilder builder,
        ReadOnlySpan<NameWithType> discovered)
        => UpdateServiceEndPoints(builder, ServiceEndPointType.Client, discovered);

    internal void UpdateActionServers(
        IGraphBuilder builder,
        ReadOnlySpan<NameWithType> discovered)
        => UpdateActionEndPoints(builder, ActionEndPointType.Server, discovered);

    internal void UpdateActionClients(
        IGraphBuilder builder,
        ReadOnlySpan<NameWithType> discovered)
        => UpdateActionEndPoints(builder, ActionEndPointType.Client, discovered);

    internal void ResetSubscribers(IEnumerable<RosTopicEndPoint> subscribers)
    {
        _subscribers.Clear();
        foreach (var item in subscribers) _subscribers.Add(item);
    }

    internal void ResetPublishers(IEnumerable<RosTopicEndPoint> publishers)
    {
        _publishers.Clear();
        foreach (var item in publishers) _publishers.Add(item);
    }

    private void UpdateServiceEndPoints(
        IGraphBuilder builder,
        ServiceEndPointType type,
        ReadOnlySpan<NameWithType> discovered)
    {
        var dest = type == ServiceEndPointType.Server ? _servers : _clients;
        foreach (var svc in discovered)
        {
            if (!dest.TryGetValue(svc, out var ep))
            {
                dest[svc] = ep = new(this, ServiceEndPointType.Server, builder.GetOrAddService(svc.Name), svc.Type);

                if (type == ServiceEndPointType.Server)
                {
                    builder.OnAddServiceServer(ep);
                }
                else
                {
                    builder.OnAddServiceClient(ep);
                }
            }

            if (type == ServiceEndPointType.Server)
            {
                builder.OnEnumerateServiceServer(ep);
            }
            else
            {
                builder.OnEnumerateServiceClient(ep);
            }
        }

        foreach (var (k, _) in dest)
        {
            if (discovered.IndexOf(k) < 0)
            {
                if (dest.Remove(k, out var s))
                {
                    if (type == ServiceEndPointType.Server)
                    {
                        builder.OnRemoveServiceServer(s);
                    }
                    else
                    {
                        builder.OnRemoveServiceClient(s);
                    }
                }
            }
        }
    }

    private void UpdateActionEndPoints(
        IGraphBuilder builder,
        ActionEndPointType type,
        ReadOnlySpan<NameWithType> discovered)
    {
        var dest = type == ActionEndPointType.Server ? _actionServers : _actionClients;
        foreach (var svc in discovered)
        {
            if (!dest.TryGetValue(svc, out var ep))
            {
                dest[svc] = ep = new(this, ActionEndPointType.Server, builder.GetOrAddAction(svc.Name), svc.Type);

                if (type == ActionEndPointType.Server)
                {
                    builder.OnAddActionServer(ep);
                }
                else
                {
                    builder.OnAddActionClient(ep);
                }
            }

            if (type == ActionEndPointType.Server)
            {
                builder.OnEnumerateActionServer(ep);
            }
            else
            {
                builder.OnEnumerateActionClient(ep);
            }
        }

        foreach (var (k, _) in dest)
        {
            if (discovered.IndexOf(k) < 0)
            {
                if (dest.Remove(k, out var s))
                {
                    if (type == ActionEndPointType.Server)
                    {
                        builder.OnRemoveActionServer(s);
                    }
                    else
                    {
                        builder.OnRemoveActionClient(s);
                    }
                }
            }
        }
    }
}