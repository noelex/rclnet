using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;
using System.Text;

namespace Rcl.Internal;

internal unsafe class RclPublisher<T> : RclObject<SafePublisherHandle>, IRclPublisher<T> where T : IMessage
{
    private readonly QosProfile _actualQos;
    private readonly Encoding _textEncoding;

    public RclPublisher(
        RclNodeImpl node,
        string topicName,
        QosProfile qos,
        Encoding textEncoding)
        : base(new(node.Handle, T.GetTypeSupportHandle(), topicName, qos))
    {
        ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
            rcl_publisher_get_actual_qos(Handle.Object));
        _actualQos = QosProfile.Create(in actualQos);
        _textEncoding = textEncoding;
    }

    public QosProfile ActualQos => _actualQos;

    public unsafe string? Name
        => StringMarshal.CreatePooledString(rcl_publisher_get_topic_name(Handle.Object));

    public int Subscribers
    {
        get
        {
            size_t count;
            RclException.ThrowIfNonSuccess(
                rcl_publisher_get_subscription_count(Handle.Object, &count));
            return (int)count.Value;
        }
    }

    public bool IsValid
         => rcl_publisher_is_valid(Handle.Object);

    public void Publish(RosMessageBuffer message)
    {
        RclException.ThrowIfNonSuccess(
            rcl_publish(Handle.Object, message.Data.ToPointer(), null));
    }

    public void Publish(T message)
    {
        using var buffer = RosMessageBuffer.Create<T>();
        message.WriteTo(buffer.Data, _textEncoding);
        Publish(buffer);
    }
}
