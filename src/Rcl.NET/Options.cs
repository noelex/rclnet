using Rcl.Qos;
using Rcl.Runtime;
using System.Text;
using System.Threading.Channels;

namespace Rcl;

/// <summary>
/// Encapsulation of options for node initialization.
/// </summary>
public record NodeOptions
{
    /// <summary>
    /// Create a new <see cref="NodeOptions"/>.
    /// </summary>
    /// <param name="arguments">
    /// Arguments used to extract remappings used by the node and other ROS specific settings, as well as user defined non-ROS arguments.
    /// <para>
    /// Defaults to [].
    /// </para>
    /// </param>
    /// <param name="domaindId">
    /// If set, then this value overrides the ROS_DOMAIN_ID environment variable.
    /// <para>
    /// If unset, the node will use the ROS domain ID set in the ROS_DOMAIN_ID environment
    /// variable, or on some systems 0 if the environment variable is not set.
    /// </para>
    /// <para>
    /// Supported distro(s): =foxy
    /// </para>
    /// </param>
    /// <param name="useGlobalArguments">
    /// If <see langword="true"/> this will cause the node's behavior to be influenced by "global"
    /// arguments, i.e.arguments not targeted at specific nodes, as well as the
    /// arguments targeted at the current node.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </param>
    /// <param name="enableRosOut">
    /// If <see langword="false"/> this will cause the node not to use rosout logging.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </param>
    /// <param name="clock">
    /// <see cref="RclClock"/> to be used by the node.
    /// <para>
    /// Defaults to <see cref="RclClock.Ros"/>.
    /// </para>
    /// </param>
    /// <param name="clockQos">
    /// The QoS settings to be used for the subscriber on /clock topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.Clock"/>.
    /// </para>
    /// </param>
    /// <param name="rosOutQos">
    /// The QoS settings to be used for the publisher on /rosout topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.RosOut"/>.
    /// </para>
    /// <para>
    /// Supported distro(s): >=humble
    /// </para>
    /// </param>
    public NodeOptions(
        string[]? arguments = null,
        [SupportedDistribution(RosEnvironment.Foxy)]
        nuint? domaindId = null,
        bool useGlobalArguments = true,
        bool enableRosOut = true,
        RclClock? clock = null,
        QosProfile? clockQos = null,
        [SupportedSinceDistribution(RosEnvironment.Humble)]
        QosProfile? rosOutQos = null)
    {
        if (domaindId != null)
        {
            RosEnvironment.Require(RosEnvironment.Foxy, VersionRequirement.Exact, nameof(DomaindId));
        }

        if (rosOutQos != null)
        {
            RosEnvironment.Require(RosEnvironment.Humble, feature: nameof(RosOutQos));
        }

        Arguments = arguments ?? Array.Empty<string>();
        DomaindId = domaindId;
        UseGlobalArguments = useGlobalArguments;
        EnableRosOut = enableRosOut;
        Clock = clock ?? RclClock.Ros;
        ClockQos = clockQos ?? QosProfile.Clock;
        RosOutQos = rosOutQos ?? QosProfile.RosOut;
    }

    /// <summary>
    /// Arguments used to extract remappings used by the node and other ROS specific settings, as well as user defined non-ROS arguments.
    /// <para>
    /// Defaults to [].
    /// </para>
    /// </summary>
    public string[] Arguments { get; }

    /// <summary>
    /// If set, then this value overrides the ROS_DOMAIN_ID environment variable.
    /// <para>
    /// If unset, the node will use the ROS domain ID set in the ROS_DOMAIN_ID environment
    /// variable, or on some systems 0 if the environment variable is not set.
    /// </para>
    /// <para>
    /// Supported by: =foxy
    /// </para>
    /// </summary>
    [SupportedDistribution(RosEnvironment.Foxy)]
    public nuint? DomaindId { get; }

    /// <summary>
    /// If <see langword="true"/> this will cause the node's behavior to be influenced by "global"
    /// arguments, i.e.arguments not targeted at specific nodes, as well as the
    /// arguments targeted at the current node.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </summary>
    public bool UseGlobalArguments { get; }

    /// <summary>
    /// If <see langword="false"/> this will cause the node not to use rosout logging.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </summary>
    public bool EnableRosOut { get; }

    /// <summary>
    /// The QoS settings to be used for the publisher on /rosout topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.RosOut"/>.
    /// </para>
    /// </summary>
    /// <remarks>
    /// Supported distro(s): >=humble
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    public QosProfile RosOutQos { get; }

    /// <summary>
    /// <see cref="RclClock"/> to be used by the node.
    /// <para>
    /// Defaults to <see cref="RclClock.Ros"/>.
    /// </para>
    /// </summary>
    public RclClock Clock { get; }

    /// <summary>
    /// The QoS settings to be used for the subscriber on /clock topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.Clock"/>.
    /// </para>
    /// </summary>
    public QosProfile ClockQos { get; }

    /// <summary>
    /// Gets a <see cref="NodeOptions"/> instance with all options set to default values.
    /// </summary>
    public static NodeOptions Default { get; } = new();
}

/// <summary>
/// Extra options for creating subscriptions.
/// </summary>
public record SubscriptionOptions
{
    /// <summary>
    /// Create a new <see cref="SubscriptionOptions"/>.
    /// </summary>
    /// <param name="qos">
    /// The QoS settings to be used for the subscribing the topic.
    /// <para>
    /// Defaults to <see cref="QosProfile.Default"/>.
    /// </para>
    /// </param>
    /// <param name="textEncoding">
    /// Encoding of the string values in the message.
    /// <para>This setting only applies to messages received as POCOs, not native message buffers.</para>
    /// </param>
    /// <param name="queueSize">
    /// Capacity of the message delivery queue.
    /// This parameter is used for setting up the queue for devilering messages via <see cref="IAsyncEnumerable{T}"/>s,
    /// and is irrelevant to <see cref="QosProfile.Depth"/>.
    /// </param>
    /// <param name="fullMode">
    /// Behavior to use when the delivery queue is full.
    /// </param>
    /// <param name="livelinessChangedHandler">
    /// Handler for receiving <see cref="LivelinessChangedEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the subscription,
    /// depending on whether the underlying RMW implementation supports <see cref="LivelinessChangedEvent"/>.
    /// </para>
    /// </param>
    /// <param name="requestedDeadlineMissedHandler">
    /// Handler for receiving <see cref="RequestedDeadlineMissedEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the subscription,
    /// depending on whether the underlying RMW implementation supports <see cref="RequestedDeadlineMissedEvent"/>.
    /// </para>
    /// </param>
    /// <param name="requestedQosIncompatibleHandler">
    /// Handler for receiving <see cref="IncompatibleQosEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the subscription,
    /// depending on whether the underlying RMW implementation supports <see cref="IncompatibleQosEvent"/>.
    /// </para>
    /// </param>
    public SubscriptionOptions(
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        Action<LivelinessChangedEvent>? livelinessChangedHandler = null,
        Action<RequestedDeadlineMissedEvent>? requestedDeadlineMissedHandler = null,
        Action<IncompatibleQosEvent>? requestedQosIncompatibleHandler = null)
    {
        Qos = qos ?? QosProfile.Default;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        QueueSize = queueSize;
        FullMode = fullMode;
        LivelinessChangedHandler = livelinessChangedHandler;
        RequestedDeadlineMissedHandler = requestedDeadlineMissedHandler;
        RequestedQosIncompatibleHandler = requestedQosIncompatibleHandler;
    }

    /// <summary>
    /// Gets a <see cref="SubscriptionOptions"/> instance with with all options set to default values.
    /// </summary>
    public static SubscriptionOptions Default { get; } = new();

    /// <summary>
    /// The QoS settings to be used for the subscribing the topic.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.Default"/>.
    /// </remarks>
    public QosProfile Qos { get; }

    /// <summary>
    /// Encoding of the string values in the message.
    /// </summary>
    /// <remarks>This setting only applies to messages received as POCOs, not native message buffers.</remarks>
    public Encoding TextEncoding { get; }

    /// <summary>
    /// Capacity of the message delivery queue.
    /// This parameter is used for setting up the queue for devilering messages via <see cref="IAsyncEnumerable{T}"/>s,
    /// and is irrelevant to <see cref="QosProfile.Depth"/>.
    /// </summary>
    public int QueueSize { get; }

    /// <summary>
    /// Behavior to use when the delivery queue is full.
    /// </summary>
    public BoundedChannelFullMode FullMode { get; }

    /// <summary>
    /// Handler for receiving <see cref="LivelinessChangedEvent"/>s.
    /// </summary>
    /// <remarks>
    /// Setting this option may cause <see cref="RclException"/> when creating the subscription,
    /// depending on whether the underlying RMW implementation supports <see cref="LivelinessChangedEvent"/>.
    /// </remarks>
    public Action<LivelinessChangedEvent>? LivelinessChangedHandler { get; }

    /// <summary>
    /// Handler for receiving <see cref="RequestedDeadlineMissedEvent"/>s.
    /// </summary>
    /// <remarks>
    /// Setting this option may cause <see cref="RclException"/> when creating the subscription,
    /// depending on whether the underlying RMW implementation supports <see cref="RequestedDeadlineMissedEvent"/>.
    /// </remarks>
    public Action<RequestedDeadlineMissedEvent>? RequestedDeadlineMissedHandler { get; }

    /// <summary>
    /// Handler for receiving <see cref="IncompatibleQosEvent"/>s.
    /// </summary>
    /// <remarks>
    /// Setting this option may cause <see cref="RclException"/> when creating the subscription,
    /// depending on whether the underlying RMW implementation supports <see cref="IncompatibleQosEvent"/>.
    /// </remarks>
    public Action<IncompatibleQosEvent>? RequestedQosIncompatibleHandler { get; }
}

/// <summary>
/// Extra options for creating publishers.
/// </summary>
public record PublisherOptions
{
    /// <summary>
    /// Create a new <see cref="PublisherOptions"/>.
    /// </summary>
    /// <param name="qos">
    /// The QoS settings to be used for the publishing the topic.
    /// <para>
    /// Defaults to <see cref="QosProfile.Default"/>.
    /// </para>
    /// </param>
    /// <param name="textEncoding">
    /// Encoding of the string values in the message.
    /// <para>This setting only applies to messages published as POCOs, not native message buffers.</para>
    /// </param>
    /// <param name="livelinessLostHandler">
    /// Handler for receiving <see cref="LivelinessLostEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="LivelinessLostEvent"/>.
    /// </param>
    /// <param name="offeredDeadlineMissedHandler">
    /// Handler for receiving <see cref="OfferedDeadlineMissedEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="OfferedDeadlineMissedEvent"/>.
    /// </param>
    /// <param name="offeredQosIncompatibleHandler">
    /// Handler for receiving <see cref="IncompatibleQosEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="IncompatibleQosEvent"/>.
    /// </param>
    public PublisherOptions(
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        Action<LivelinessLostEvent>? livelinessLostHandler = null,
        Action<OfferedDeadlineMissedEvent>? offeredDeadlineMissedHandler = null,
        Action<IncompatibleQosEvent>? offeredQosIncompatibleHandler = null)
    {
        Qos = qos ?? QosProfile.Default;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        LivelinessLostHandler = livelinessLostHandler;
        OfferedDeadlineMissedHandler = offeredDeadlineMissedHandler;
        OfferedQosIncompatibleHandler = offeredQosIncompatibleHandler;
    }

    /// <summary>
    /// Gets a <see cref="SubscriptionOptions"/> instance with with all options set to default values.
    /// </summary>
    public static PublisherOptions Default { get; } = new();

    /// <summary>
    /// The QoS settings to be used for the publishing the topic.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.Default"/>.
    /// </remarks>
    public QosProfile Qos { get; }

    /// <summary>
    /// Encoding of the string values in the message.、
    /// </summary>
    /// <remarks>This setting only applies to messages published as POCOs, not native message buffers.</remarks>
    public Encoding TextEncoding { get; }

    /// <summary>
    /// Handler for receiving <see cref="LivelinessLostEvent"/>s.
    /// </summary>
    /// <remarks>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="LivelinessLostEvent"/>.
    /// </remarks>
    public Action<LivelinessLostEvent>? LivelinessLostHandler { get; }

    /// <summary>
    /// Handler for receiving <see cref="OfferedDeadlineMissedEvent"/>s.
    /// </summary>
    /// <remarks>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="OfferedDeadlineMissedEvent"/>.
    /// </remarks>
    public Action<OfferedDeadlineMissedEvent>? OfferedDeadlineMissedHandler { get; }

    /// <summary>
    /// Handler for receiving <see cref="IncompatibleQosEvent"/>s.
    /// </summary>
    /// <remarks>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="IncompatibleQosEvent"/>.
    /// </remarks>
    public Action<IncompatibleQosEvent>? OfferedQosIncompatibleHandler { get; }
}

/// <summary>
/// Extra options for creating service servers.
/// </summary>
public record ServerOptions
{
    /// <summary>
    /// Create a new <see cref="ServerOptions"/>.
    /// </summary>
    /// <param name="qos">
    /// The QoS settings to be used for the receiving and sending service requests/responses.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="textEncoding">
    /// Encoding of the string values in the message.
    /// <para>This setting only applies to messages received/sent as POCOs, not native message buffers.</para>
    /// </param>
    public ServerOptions(QosProfile? qos = null, Encoding? textEncoding = null)
    {
        Qos = qos ?? QosProfile.ServicesDefault;
        TextEncoding = textEncoding ?? Encoding.UTF8;
    }

    /// <summary>
    /// Gets a <see cref="ServerOptions"/> instance with with all options set to default values.
    /// </summary>
    public static ServerOptions Default { get; } = new();

    /// <summary>
    /// The QoS settings to be used for the receiving and sending service requests/responses.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile Qos { get; }

    /// <summary>
    /// Encoding of the string values in the message.
    /// </summary>
    /// <remarks>
    /// This setting only applies to messages received/sent as POCOs, not native message buffers.
    /// </remarks>
    public Encoding TextEncoding { get; }
}

/// <summary>
/// Extra options for creating service clients.
/// </summary>
public record ClientOptions
{
    /// <summary>
    /// Create a new <see cref="ClientOptions"/>.
    /// </summary>
    /// <param name="qos">
    /// The QoS settings to be used for the sending and receiving service requests/responses.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="textEncoding">
    /// Encoding of the string values in the message.
    /// <para>This setting only applies to messages sent/received as POCOs, not native message buffers.</para>
    /// </param>
    public ClientOptions(QosProfile? qos = null, Encoding? textEncoding = null)
    {
        Qos = qos ?? QosProfile.ServicesDefault;
        TextEncoding = textEncoding ?? Encoding.UTF8;
    }

    /// <summary>
    /// Gets a <see cref="ClientOptions"/> instance with with all options set to default values.
    /// </summary>
    public static ClientOptions Default { get; } = new();

    /// <summary>
    /// The QoS settings to be used for the sending and receiving service requests/responses.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile Qos { get; }

    /// <summary>
    /// Encoding of the string values in the message.
    /// </summary>
    /// <remarks>
    /// This setting only applies to messages sent/received as POCOs, not native message buffers.
    /// </remarks>
    public Encoding TextEncoding { get; }
}