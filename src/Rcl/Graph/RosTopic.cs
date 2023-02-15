using System.Collections.Concurrent;

namespace Rcl.Graph;

/// <summary>
/// Represents a topic in the ROS graph.
/// </summary>
public class RosTopic
{
    private readonly ConcurrentDictionary<GraphId, RosTopicEndPoint>
        _publishers = new(), _subscribers = new();

    internal RosTopic(string name)
    {
        Name = name;
        Publishers = ReadOnlyCollection.Wrap(_publishers.Values);
        Subscribers = ReadOnlyCollection.Wrap(_subscribers.Values);
    }

    /// <summary>
    /// Gets the name of the ROS topic.
    /// </summary>
    public string Name { get; private set; }

    public override string ToString()
    {
        return Name;
    }

    /// <summary>
    /// Gets a list of available ROS topic publishers registered with current <see cref="RosTopic"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Publishers { get; }

    /// <summary>
    /// Gets a list of available ROS topic subscribers registered with current <see cref="RosTopic"/>.
    /// </summary>
    public IReadOnlyCollection<RosTopicEndPoint> Subscribers { get; }

    internal void UpdatePublishers(
        IGraphBuilder builder,
        ReadOnlySpan<TopicEndPointData> endpoints,
        IDictionary<NodeName, RosNode> nodes)
    {
        UpdateEndPoints(builder, TopicEndPointType.Publisher, endpoints, nodes);
    }

    internal void UpdateSubscribers(
        IGraphBuilder builder,
        ReadOnlySpan<TopicEndPointData> endpoints,
        IDictionary<NodeName, RosNode> nodes)
    {
        UpdateEndPoints(builder, TopicEndPointType.Subscriber, endpoints, nodes);
    }

    private void UpdateEndPoints(
        IGraphBuilder builder,
        TopicEndPointType type,
        ReadOnlySpan<TopicEndPointData> endpoints,
        IDictionary<NodeName, RosNode> nodes)
    {
        var dest = type == TopicEndPointType.Publisher ? _publishers : _subscribers;
        foreach (var ep in endpoints)
        {
            // Endpoints may establish before node establishes.
            // If this happends, simply ignore it here and add the endpoint
            // when node is established.
            if (!dest.TryGetValue(ep.Id, out _) &&
                nodes.TryGetValue(ep.Node, out var node))
            {
                var endpoint = new RosTopicEndPoint(
                    this, node, type,  ep.Type, ep.QosProfile);

                dest[ep.Id] = endpoint;
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

        foreach (var (id, v) in dest)
        {
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
            else
            {
                if (type is TopicEndPointType.Publisher)
                {
                    builder.OnEnumeratePublisher(v);
                }
                else
                {
                    builder.OnEnumerateSubscriber(v);
                }
            }
        }
    }
}