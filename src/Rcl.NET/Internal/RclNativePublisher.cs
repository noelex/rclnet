using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime.Interop;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using Rcl.Introspection;

namespace Rcl.Internal;

internal unsafe class RclNativePublisher : RclObject<SafePublisherHandle>, IRclPublisher
{
    private readonly QosProfile _actualQos;
    private readonly IMessageIntrospection _introspection;

    public RclNativePublisher(
        RclNodeImpl node,
        string topicName,
        TypeSupportHandle typesupport,
        QosProfile qos)
        : base(new(node.Handle, typesupport, topicName, qos))
    {
        ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
            rcl_publisher_get_actual_qos(Handle.Object));
        _actualQos = QosProfile.Create(in actualQos);

        _introspection = MessageIntrospection.Create(typesupport);
        Name = StringMarshal.CreatePooledString(rcl_publisher_get_topic_name(Handle.Object))!;
    }

    public QosProfile ActualQos => _actualQos;

    public string Name { get; }

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

    public RosMessageBuffer CreateBuffer() => _introspection.CreateBuffer();
}