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
using Rcl.Internal.Events;
using Rcl.Logging;

namespace Rcl.Internal;

internal unsafe class RclNativePublisher : RclObject<SafePublisherHandle>, IRclPublisher
{
    private readonly QosProfile _actualQos;
    private readonly IMessageIntrospection _introspection;
    private readonly RclNodeImpl _node;

    private readonly RclPubisherEvent? _livelinessEvent, _deadlineMissedEvent, _qosEvent;

    public RclNativePublisher(
        RclNodeImpl node,
        string topicName,
        TypeSupportHandle typesupport,
        PublisherOptions options)
        : base(new(node.Handle, typesupport, topicName, options.Qos))
    {
        _node = node;
        ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
            rcl_publisher_get_actual_qos(Handle.Object));
        _actualQos = QosProfile.Create(in actualQos);

        _introspection = MessageIntrospection.Create(typesupport);
        Name = StringMarshal.CreatePooledString(rcl_publisher_get_topic_name(Handle.Object))!;
        Options = options;

        try
        {
            _livelinessEvent = new RclPublisherLivelinessLostEvent(
                node.Context, Handle,
                options.LivelinessLostHandler ?? OnLivelinessEvent);
        }
        catch (RclException ex)
        {
            if (options.LivelinessLostHandler != null)
            {
                _node.Context.DefaultLogger.LogWarning("Unable to register LivelinessLostEvent:");
                _node.Context.DefaultLogger.LogWarning(ex.Message);
            }
        }

        try
        {
            _deadlineMissedEvent = new RclPublisherOfferedDeadlineMissedEvent(
                node.Context, Handle,
                options.OfferedDeadlineMissedHandler ?? OnDeadlineEvent);
        }
        catch (RclException ex)
        {
            if(options.OfferedDeadlineMissedHandler != null)
            {
                _node.Context.DefaultLogger.LogWarning("Unable to register OfferedDeadlineMissedEvent:");
                _node.Context.DefaultLogger.LogWarning(ex.Message);
            }
        }

        try
        {
            _qosEvent = new RclPublisherIncompatibleQosEvent(
                node.Context, Handle,
                options.OfferedQosIncompatibleHandler ?? OnIncompatibleQosEvent);
        }
        catch (RclException ex)
        {
            if(options.OfferedQosIncompatibleHandler != null)
            {
                _node.Context.DefaultLogger.LogWarning("Unable to register OfferedQosIncompatibleEvent:");
                _node.Context.DefaultLogger.LogWarning(ex.Message);
            }
        }
    }

    private void OnLivelinessEvent(LivelinessLostEvent info)
    {
        _node.Logger.LogWarning(
            $"Received LivelinessLostEvent on publisher of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}");
    }

    private void OnDeadlineEvent(OfferedDeadlineMissedEvent info)
    {
        _node.Logger.LogWarning(
            $"Received OfferedDeadlineMissedEvent on publisher of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}");
    }

    private void OnIncompatibleQosEvent(IncompatibleQosEvent info)
    {
        _node.Logger.LogWarning(
            $"Received IncompatibleQosEvent on publisher of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}, PolicyKind = {info.LastPolicyKind}");
    }

    public PublisherOptions Options { get; }

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

    public override void Dispose()
    {
        _livelinessEvent?.Dispose();
        _deadlineMissedEvent?.Dispose();
        _qosEvent?.Dispose();

        base.Dispose();
    }
}