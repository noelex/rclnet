using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a topic in the ROS graph.
/// </summary>
public class RosTopic
{
    private readonly ConcurrentDictionary<GraphId, RosTopicEndPoint>
        _publishers = new(), _subscribers = new();
    private readonly IEnumerator<KeyValuePair<GraphId, RosTopicEndPoint>>
        _publishersEnumerator, _subscribersEnumerator;

    internal RosTopic(string name)
    {
        Name = name;
        _publishersEnumerator = _publishers.GetEnumerator();
        _subscribersEnumerator = _subscribers.GetEnumerator();
    }

    /// <summary>
    /// Gets the name of the ROS topic.
    /// </summary>
    public string Name { get; private set; }

    /// <inheritdoc/>
    public override string ToString()
    {
        return Name;
    }

    /// <summary>
    /// Gets a list of available ROS topic publishers registered with current <see cref="RosTopic"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Publishers => (IReadOnlyCollection<RosTopicEndPoint>)_publishers.Values;

    /// <summary>
    /// Gets a list of available ROS topic subscribers registered with current <see cref="RosTopic"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Subscribers => (IReadOnlyCollection<RosTopicEndPoint>)_subscribers.Values;

    internal void UpdatePublishers(
        IGraphBuilder builder,
        ReadOnlySpan<TopicEndPointDataRef> endpoints,
        ConcurrentDictionary<string, RosNode> nodes)
    {
        UpdateEndPoints(builder, TopicEndPointType.Publisher, endpoints, nodes);
    }

    internal void UpdateSubscribers(
        IGraphBuilder builder,
        ReadOnlySpan<TopicEndPointDataRef> endpoints,
        ConcurrentDictionary<string, RosNode> nodes)
    {
        UpdateEndPoints(builder, TopicEndPointType.Subscriber, endpoints, nodes);
    }

    private void UpdateEndPoints(
        IGraphBuilder builder,
        TopicEndPointType type,
        ReadOnlySpan<TopicEndPointDataRef> endpoints,
        ConcurrentDictionary<string, RosNode> nodes)
    {
        var (dest, destEnumerator) = type == TopicEndPointType.Publisher
            ? (_publishers, _publishersEnumerator)
            : (_subscribers, _subscribersEnumerator);
        foreach (var ep in endpoints)
        {
            // Endpoints may establish before node establishes.
            // If this happends, simply ignore it here and add the endpoint
            // when node is established.
            var id = ep.Id;
            if (!dest.TryGetValue(id, out _) &&
                nodes.TryGetValue(ep.Node.FullyQualifiedName, out var node))
            {
                var endpoint = new RosTopicEndPoint(
                   id, this, node, type, ep.Type, ep.QosProfile);

                dest[id] = endpoint;
                if (type is TopicEndPointType.Publisher)
                {
                    builder.OnAddPublisher(endpoint);
                }
                else
                {
                    builder.OnAddSubscriber(endpoint);
                }
            }
        }

        try
        {
            while (destEnumerator.MoveNext())
            {
                var (id, v) = destEnumerator.Current;
                bool found = false;
                foreach (var ep in endpoints)
                {
                    if (ep.Id == id)
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    if (dest.Remove(id, out var s))
                    {
                        if (type is TopicEndPointType.Publisher)
                        {
                            builder.OnRemovePublisher(s);
                        }
                        else
                        {
                            builder.OnRemoveSubscriber(s);
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