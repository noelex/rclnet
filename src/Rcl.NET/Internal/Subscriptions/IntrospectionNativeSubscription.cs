using Rcl.Interop;
using Rcl.Introspection;
using Rosidl.Runtime;

namespace Rcl.Internal.Subscriptions;

/// <summary>
/// Use introspection to create message buffers, without specifying the message type as a generic argument.
/// </summary>
internal unsafe class IntrospectionNativeSubscription
    : NativeSubscriptionBase, IRclNativeSubscription
{
    private readonly IMessageIntrospection _introspection;

    public IntrospectionNativeSubscription(
        RclNodeImpl node,
        string topicName,
        TypeSupportHandle typeSupport,
        SubscriptionOptions options)
        : base(node, topicName, typeSupport, options)
    {
        _introspection = MessageIntrospection.Create(typeSupport);
    }

    protected override unsafe RosMessageBuffer TakeMessage()
    {
        // TODO: Parse this as RclFoxy.rmw_message_info_t
        // if need to access header fields on foxy.
        // Defined as RclHumble.rmw_message_info_t only because it has bigger size
        // to be compatible with both foxy and humble.
        //
        // Size of the GID was changed from 24 bytes to 16 bytes since iron.
        // We can still use RclHumble.rmw_message_info_t because it's bigger in size,
        // unless we need to access rmw_message_info_t.from_intra_process.
        RclHumble.rmw_message_info_t header;

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
