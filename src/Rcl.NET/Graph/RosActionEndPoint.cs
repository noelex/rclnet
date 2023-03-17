namespace Rcl.Graph;

/// <summary>
/// Represents the type of an action endpoint.
/// </summary>
public enum ActionEndPointType
{
    /// <summary>
    /// The endpoint is an action server.
    /// </summary>
    Server,

    /// <summary>
    /// The endpoint is an action client.
    /// </summary>
    Client
}

/// <summary>
/// Represents an endpoint of an ROS action.
/// </summary>
/// <param name="Node">The <see cref="RosNode"/> which is hosting the endpoint.</param>
/// <param name="EndPointType">The type of an action endpoint.</param>
/// <param name="Action">The <see cref="RosAction"/> which the endpoint belongs to.</param>
/// <param name="Type">The type of the action.</param>
public record RosActionEndPoint
    (
        RosNode Node,
        ActionEndPointType EndPointType,
        RosAction Action,
        string Type
    )
{
    /// <inheritdoc/>
    public override string ToString()
    {
        return $"Action{Enum.GetName(EndPointType)}(Action = {Action}, Type = {Type})";
    }
}