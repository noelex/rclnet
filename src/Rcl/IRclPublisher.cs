using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl;

public interface IRclPublisher<T> : IRclObject
    where T : IMessage
{
    QosProfile ActualQos { get; }
    bool IsValid { get; }
    int Subscribers { get; }
    string? Name { get; }

    void Publish(RosMessageBuffer message);

    void Publish(T message);
}