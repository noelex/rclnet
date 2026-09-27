using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a node in the ROS graph.
/// </summary>
public class RosNode
{
    private IReadOnlyCollection<RosServiceEndPoint> _serversSnapshot = Array.Empty<RosServiceEndPoint>();
    private IReadOnlyCollection<RosServiceEndPoint> _clientsSnapshot = Array.Empty<RosServiceEndPoint>();
    private IReadOnlyCollection<RosTopicEndPoint> _publishersSnapshot = Array.Empty<RosTopicEndPoint>();
    private IReadOnlyCollection<RosTopicEndPoint> _subscribersSnapshot = Array.Empty<RosTopicEndPoint>();
    private IReadOnlyCollection<RosActionEndPoint> _actionServersSnapshot = Array.Empty<RosActionEndPoint>();
    private IReadOnlyCollection<RosActionEndPoint> _actionClientsSnapshot = Array.Empty<RosActionEndPoint>();

    private readonly ConcurrentDictionary<NameWithType, RosServiceEndPoint>
        _servers = new(), _clients = new();

    private readonly ConcurrentDictionary<RosTopicEndPoint, RosTopicEndPoint>
        _subscribers = new(), _publishers = new();

    private readonly ConcurrentDictionary<NameWithType, RosActionEndPoint>
        _actionServers = new(), _actionClients = new();

    private readonly IEnumerator<KeyValuePair<NameWithType, RosServiceEndPoint>>
        _serversEnumerator, _clientsEnumerator;

    private readonly IEnumerator<KeyValuePair<NameWithType, RosActionEndPoint>>
        _actionServersEnumerator, _actionClientsEnumerator;

    private readonly IEnumerator<KeyValuePair<RosTopicEndPoint, RosTopicEndPoint>>
        _subscribersEnumerator, _publishersEnumerator;

    internal void PublishSnapshots()
    {
        Volatile.Write(ref _serversSnapshot, (IReadOnlyCollection<RosServiceEndPoint>)_servers.Values);
        Volatile.Write(ref _clientsSnapshot, (IReadOnlyCollection<RosServiceEndPoint>)_clients.Values);
        Volatile.Write(ref _publishersSnapshot, (IReadOnlyCollection<RosTopicEndPoint>)_publishers.Values);
        Volatile.Write(ref _subscribersSnapshot, (IReadOnlyCollection<RosTopicEndPoint>)_subscribers.Values);
        Volatile.Write(ref _actionServersSnapshot, (IReadOnlyCollection<RosActionEndPoint>)_actionServers.Values);
        Volatile.Write(ref _actionClientsSnapshot, (IReadOnlyCollection<RosActionEndPoint>)_actionClients.Values);
    }

    internal RosNode(NodeName name, string enclave)
    {
        Name = name;
        Enclave = enclave;

        _serversEnumerator = _servers.GetEnumerator();
        _clientsEnumerator = _clients.GetEnumerator();
        _actionServersEnumerator = _actionServers.GetEnumerator();
        _actionClientsEnumerator = _actionClients.GetEnumerator();
        _subscribersEnumerator = _subscribers.GetEnumerator();
        _publishersEnumerator = _publishers.GetEnumerator();
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
    public IReadOnlyCollection<RosServiceEndPoint> Servers => Volatile.Read(ref _serversSnapshot);

    /// <summary>
    /// Gets a list of <see cref="RosService"/> clients registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosServiceEndPoint> Clients => Volatile.Read(ref _clientsSnapshot);

    /// <summary>
    /// Gets a list of <see cref="RosTopic"/> subscribers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Subscribers => Volatile.Read(ref _subscribersSnapshot);

    /// <summary>
    /// Gets a list of <see cref="RosTopic"/> publishers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Publishers => Volatile.Read(ref _publishersSnapshot);

    /// <summary>
    /// Gets a list of <see cref="RosAction"/> servers registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> ActionServers => Volatile.Read(ref _actionServersSnapshot);

    /// <summary>
    /// Gets a list of <see cref="RosAction"/> clients registered by current <see cref="RosNode"/>.
    /// </summary>
    public IReadOnlyCollection<RosActionEndPoint> ActionClients => Volatile.Read(ref _actionClientsSnapshot);

    internal IEnumerator<KeyValuePair<RosTopicEndPoint, RosTopicEndPoint>> PublishersEnumerator => _publishersEnumerator;

    internal IEnumerator<KeyValuePair<RosTopicEndPoint, RosTopicEndPoint>> SubscribersEnumerator => _subscribersEnumerator;

    internal int PublisherCount => _publishers.Count;

    internal int SubscriberCount => _subscribers.Count;

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

    internal void RemoveSubscriber(RosTopicEndPoint subscriber)
    {
        _subscribers.Remove(subscriber, out _);
    }

    internal void RemovePublisher(RosTopicEndPoint publisher)
    {
        _publishers.Remove(publisher, out _);
    }

    internal void AddSubscriber(RosTopicEndPoint subscriber)
    {
        _subscribers[subscriber] = subscriber;
    }

    internal void AddPublisher(RosTopicEndPoint publisher)
    {
        _publishers[publisher] = publisher;
    }

    internal void ResetSubscribers(Span<RosTopicEndPoint> subscribers)
    {
        try
        {
            while (_subscribersEnumerator.MoveNext())
            {
                var k = _subscribersEnumerator.Current.Key;
                if (!subscribers.Contains(k))
                {
                    _subscribers.Remove(k, out _);
                }
            }
        }
        finally
        {
            _subscribersEnumerator.Reset();
        }

        foreach (var item in subscribers) _subscribers[item] = item;
    }

    internal void ResetPublishers(Span<RosTopicEndPoint> publishers)
    {
        try
        {
            while (_publishersEnumerator.MoveNext())
            {
                var k = _publishersEnumerator.Current.Key;
                if (!publishers.Contains(k))
                {
                    _publishers.Remove(k, out _);
                }
            }
        }
        finally
        {
            _publishersEnumerator.Reset();
        }

        foreach (var item in publishers) _publishers[item] = item;
    }

    private void UpdateServiceEndPoints(
        IGraphBuilder builder,
        ServiceEndPointType type,
        ReadOnlySpan<NameWithType> discovered)
    {
        var (dest, destEnumerator) = type == ServiceEndPointType.Server
            ? (_servers, _serversEnumerator)
            : (_clients, _clientsEnumerator);
        foreach (var svc in discovered)
        {
            if (!dest.TryGetValue(svc, out var ep))
            {
                dest[svc] = ep = new(this, type, builder.GetOrAddService(svc.Name), svc.Type);

                if (type == ServiceEndPointType.Server)
                {
                    builder.OnAddServiceServer(ep);
                }
                else
                {
                    builder.OnAddServiceClient(ep);
                }
            }
        }

        try
        {
            while (destEnumerator.MoveNext())
            {
                var k = destEnumerator.Current.Key;
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
        finally
        {
            destEnumerator.Reset();
        }
    }

    private void UpdateActionEndPoints(
        IGraphBuilder builder,
        ActionEndPointType type,
        ReadOnlySpan<NameWithType> discovered)
    {
        var (dest, destEnumerator) = type == ActionEndPointType.Server
            ? (_actionServers, _actionServersEnumerator)
            : (_actionClients, _actionClientsEnumerator);
        foreach (var svc in discovered)
        {
            if (!dest.TryGetValue(svc, out var ep))
            {
                dest[svc] = ep = new(this, type, builder.GetOrAddAction(svc.Name), svc.Type);

                if (type == ActionEndPointType.Server)
                {
                    builder.OnAddActionServer(ep);
                }
                else
                {
                    builder.OnAddActionClient(ep);
                }
            }
        }

        try
        {
            while (destEnumerator.MoveNext())
            {
                var k = destEnumerator.Current.Key;
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
        finally
        {
            destEnumerator.Reset();
        }
    }
}