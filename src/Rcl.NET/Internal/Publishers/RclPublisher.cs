using Rosidl.Runtime;

namespace Rcl.Internal.Publishers;

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

    public ValueTask PublishAsync(T message)
    {
        var buffer = CreateBuffer();
        message.WriteTo(buffer.Data, Options.TextEncoding);
        return PublishAsync(buffer, true);
    }
}
