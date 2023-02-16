using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl;

public interface IRclPublisher : IRclObject
{
    QosProfile ActualQos { get; }

    bool IsValid { get; }

    int Subscribers { get; }

    string? Name { get; }

    void Publish(RosMessageBuffer message);

    RosMessageBuffer CreateBuffer();
}

public interface IRclPublisher<T> : IRclPublisher
    where T : IMessage
{
    void Publish(T message);
}