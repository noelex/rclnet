using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;
using System.Text;

namespace Rcl.Internal;

internal unsafe class RclPublisher<T> : RclNativePublisher, IRclPublisher<T> where T : IMessage
{
    public RclPublisher(
        RclNodeImpl node,
        string topicName,
        PublisherOptions options)
        : base(node, topicName, T.GetTypeSupportHandle(), options)
    {
    }

    public void Publish(T message)
    {
        using var buffer = CreateBuffer();
        message.WriteTo(buffer.Data, Options.TextEncoding);
        Publish(buffer);
    }
}
