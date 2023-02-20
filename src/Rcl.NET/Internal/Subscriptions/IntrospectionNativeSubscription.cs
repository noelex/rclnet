using Rcl.Interop;
using Rcl.Introspection;
using Rcl.Qos;
using Rosidl.Runtime;
using System.Threading.Channels;

namespace Rcl.Internal.Subscriptions;

/// <summary>
/// Use introspection to create message buffers, without specifying the message type as a generic argument.
/// </summary>
internal unsafe class IntrospectionNativeSubscription
    : NativeSubscriptionBase, IRclNativeSubscription
{
    private readonly MessageIntrospection _introspection;

    public IntrospectionNativeSubscription(
        RclNodeImpl node,
        string topicName,
        TypeSupportHandle typeSupport,
        QosProfile qos,
        int queueSize,
        BoundedChannelFullMode fullMode)
        : base(node, topicName, typeSupport, qos, queueSize, fullMode)
    {
        _introspection = new MessageIntrospection(typeSupport);
    }

    protected override unsafe RosMessageBuffer TakeMessage()
    {
        rmw_message_info_t header;
        var buffer = _introspection.CreateBuffer();

        if (rcl_ret_t.RCL_RET_OK ==
            rcl_take(Handle.Object, buffer.Data.ToPointer(), &header, null))
        {
            return buffer;
        }
        else
        {
            buffer.Dispose();
            return RosMessageBuffer.Empty;
        }
    }
}
