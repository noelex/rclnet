using Rcl.Qos;

namespace Rcl.Graph;

record struct NameWithType(string Name, string Type);

unsafe interface ITopicEndPointDataAccessor
{
    GraphId GetGraphId(void* data);

    string GetType(void* data);

    NodeName GetNodeName(void* data);

    QosProfile GetQosProfile(void* data);
}

unsafe readonly struct TopicEndPointDataRef
{
    private readonly void* _data;
    private readonly ITopicEndPointDataAccessor _accessor;

    public TopicEndPointDataRef(void* data, ITopicEndPointDataAccessor accessor)
    {
        _data = data;
        _accessor = accessor;
    }

    public readonly GraphId Id => _accessor.GetGraphId(_data);

    public readonly string Type => _accessor.GetType(_data);

    public readonly NodeName Node => _accessor.GetNodeName(_data);

    public readonly QosProfile QosProfile => _accessor.GetQosProfile(_data);

}