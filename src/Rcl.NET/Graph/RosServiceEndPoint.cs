namespace Rcl.Graph;

/// <summary>
/// Type of the service endpoint.
/// </summary>
public enum ServiceEndPointType
{
    /// <summary>
    /// The endpoint is a service server.
    /// </summary>
    Server,

    /// <summary>
    /// The endpoint is a service client.
    /// </summary>
    Client
}

/// <summary>
/// Represents an endpoint of a ROS service.
/// </summary>
/// <param name="Node">The node which created the endpoint.</param>
/// <param name="EndPointType">Type of the service endpoint.</param>
/// <param name="Service">The service which the endpoint corresponds to.</param>
/// <param name="Type">Message type name of the endpoint.</param>
public record RosServiceEndPoint(RosNode Node, ServiceEndPointType EndPointType, RosService Service, string Type)
{
    /// <inheritdoc/>
    public override string ToString()
    {
        return $"Service{Enum.GetName(EndPointType)}(Service = {Service}, Type = {Type})";
    }
}