using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;
using System.Threading.Channels;

namespace Rcl.Internal.Subscriptions;

internal unsafe class NativeSubscription<T> :
    NativeSubscriptionBase,
    IRclNativeSubscription where T : IMessage
{
    public NativeSubscription(
        RclNodeImpl node,
        string topicName,
        SubscriptionOptions options)
        : base(node, topicName, T.GetTypeSupportHandle(), options)
    {
    }

    protected override RosMessageBuffer TakeMessage()
    {
        // TODO: Parse this as RclFoxy.rmw_message_info_t
        // if need to access header fields on foxy.
        // Defined as RclHumble.rmw_message_info_t only because it has bigger size
        // to be compatible with both foxy and humble.
        RclHumble.rmw_message_info_t header;

        var rosMessage = RosMessageBuffer.Create<T>();
        if (rcl_ret_t.RCL_RET_OK ==
            rcl_take(Handle.Object, rosMessage.Data.ToPointer(), &header, null))
        {
            return rosMessage;
        }
        else
        {
            rosMessage.Dispose();
            return RosMessageBuffer.Empty;
        }
    }
}
