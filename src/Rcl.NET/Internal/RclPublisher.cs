using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;
using System.Text;

namespace Rcl.Internal;

internal unsafe class RclPublisher<T> : RclNativePublisher, IRclPublisher<T> where T : IMessage
{
    private readonly Encoding _textEncoding;

    public RclPublisher(
        RclNodeImpl node,
        string topicName,
        QosProfile qos,
        Encoding textEncoding)
        : base(node, topicName, T.GetTypeSupportHandle(), qos)
    {
        _textEncoding = textEncoding;
    }

    public void Publish(T message)
    {
        using var buffer = CreateBuffer();
        message.WriteTo(buffer.Data, _textEncoding);
        Publish(buffer);
    }
}
