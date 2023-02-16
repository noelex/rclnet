using Rcl.Graph;

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
        return $"Action{Enum.GetName(EndPointType)}(Action = {Action}, Type = {Type})";
    }
}