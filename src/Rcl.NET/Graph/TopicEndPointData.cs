using Rcl.Qos;

namespace Rcl.Graph;

record struct NameWithType(string Name, string Type);

record struct TopicEndPointData(
    GraphId Id,
    string Type,
    NodeName Node,
    QosProfile QosProfile);