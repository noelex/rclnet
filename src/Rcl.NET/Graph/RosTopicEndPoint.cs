using Rcl.Qos;

namespace Rcl.Graph;

/// <summary>
/// Type of the topic endpoint.
/// </summary>
public enum TopicEndPointType
{
    /// <summary>
    /// The endpoint is a topic publisher.
    /// </summary>
    Publisher,

    /// <summary>
    /// The endpoint is a topic subscriber.
    /// </summary>
    Subscriber
}

/// <summary>
/// Represents an endpoint of a ROS topic.
/// </summary>
/// <param name="Topic">The topic which the endpoint is publishing or subscribed to.</param>
/// <param name="Node">The node which created the endpoint.</param>
/// <param name="EndPointType">Type of the topic endpoint.</param>
/// <param name="Type">Message type of the publication or subscription.</param>
/// <param name="QosProfile">QoS profile of  publication or subscription.</param>
public record RosTopicEndPoint(
    RosTopic Topic,
    RosNode Node,
    TopicEndPointType EndPointType,
    string Type,
    QosProfile QosProfile
)
{
    /// <inheritdoc/>
    public override string ToString()
    {
        return $"{Enum.GetName(EndPointType)}(Topic = {Topic}, Type = {Type})";
    }
}