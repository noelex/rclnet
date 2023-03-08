using Rcl.Interop;
using Rcl.Qos;
using Rcl.Runtime;
using System.Text;
using System.Threading.Channels;

namespace Rcl;

/// <summary>
/// Represents requirement on network flow endpoint uniqueness.
/// </summary>
public enum UniquenessRequirement
{
    /// <summary>
    /// Network flow endpoint uniqueness is not required.
    /// </summary>
    NotRequired = RclHumble.rmw_unique_network_flow_endpoints_requirement_t.RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED,

    /// <summary>
    /// Network flow endpoint uniqueness is strictly required.
    /// Error if not provided by RMW implementation.
    /// </summary>
    StrictlyRequired = RclHumble.rmw_unique_network_flow_endpoints_requirement_t.RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED,

    /// <summary>
    /// Network flow endpoint uniqueness is optionally required.
    /// No error if not provided RMW implementation.
    /// </summary>
    OptionallyRequired = RclHumble.rmw_unique_network_flow_endpoints_requirement_t.RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED,

    /// <summary>
    /// Network flow endpoint uniqueness requirement is decided by system.
    /// </summary>
    SystemDefault = RclHumble.rmw_unique_network_flow_endpoints_requirement_t.RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_SYSTEM_DEFAULT,
}

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
    /// Type of the <see cref="RclClock"/> to be used by the node.
    /// <para>
    /// Defaults to <see cref="RclClockType.Ros"/>.
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
        RclClockType clock = RclClockType.Ros,
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
        Clock = clock;
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
    /// Type of the <see cref="RclClock"/> to be used by the node.
    /// <para>
    /// Defaults to <see cref="RclClockType.Ros"/>.
    /// </para>
    /// </summary>
    public RclClockType Clock { get; }

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
/// Represents options to setup a content filter on subscription to receive topic messages selectively.
/// </summary>
/// <remarks>
/// Refer to <a href="https://www.omg.org/spec/DDS/1.4/PDF">DDS specification</a> <b>Annex B - Syntax for Queries and Filters</b> for details.
/// </remarks>
public record ContentFilterOptions
{
    /// <summary>
    /// Creates a new <see cref="ContentFilterOptions"/> with specified expression and arguments.
    /// </summary>
    /// <param name="expression">
    /// Specify the criteria to select the data samples of interest.
    /// 
    /// It is similar to the WHERE part of an SQL clause.
    /// </param>
    /// <param name="arguments">
    /// Give values to the tokens placeholder ‘parameters’ (i.e., "%n" tokens begin from 0) in the
    /// <paramref name="expression"/>.The number of supplied parameters must fit with the requested values.
    ///
    /// It can be an empty array if there is no "%n" tokens placeholder in <paramref name="expression"/>.
    /// Support up to 100 arguments.
    /// <para>
    /// Defaults to an empty array.
    /// </para>
    /// </param>
    public ContentFilterOptions(string expression, params string[] arguments)
    {
        if (string.IsNullOrWhiteSpace(expression))
        {
            throw new ArgumentException("Content filter expression cannot be empty.", nameof(expression));
        }

        if (arguments != null && arguments.Length > 100)
        {
            throw new ArgumentException("Content filter supports up to 100 arguments.", nameof(arguments));
        }

        Expression = expression;
        Arguments = arguments ?? Array.Empty<string>();
    }

    /// <summary>
    /// Specify the criteria to select the data samples of interest.
    /// 
    /// It is similar to the WHERE part of an SQL clause.
    /// </summary>
    public string Expression { get; }

    /// <summary>
    /// Give values to the tokens placeholder ‘parameters’ (i.e., "%n" tokens begin from 0) in the
    /// <see cref="Expression"/>.The number of supplied parameters must fit with the requested values.
    ///
    /// It can be an empty array if there is no "%n" tokens placeholder in <see cref="Expression"/>.
    /// Support up to 100 arguments.
    /// </summary>
    /// <remarks>
    /// Defaults to an empty array.
    /// </remarks>
    public string[] Arguments { get; }
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
    /// This parameter is used for setting up the queue for devilering messages via <see cref="IAsyncEnumerable{T}"/>s returned by <c>ReadAllAsync</c>,
    /// and is irrelevant to <see cref="QosProfile.Depth"/>.
    /// <para>
    /// Setting to a small queue size may cause significant message drop under very high publishing rate,
    /// especially when <paramref name="allowSynchronousContinuations"/> is set to <see langword="false"/>.
    /// </para>
    /// </param>
    /// <param name="fullMode">
    /// Behavior to use when the delivery queue is full.
    /// <para>
    /// <see cref="BoundedChannelFullMode.Wait"/> is not supported as it will cause the event loop to be blocked when the message delivery queue is full.
    /// </para>
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
    /// <param name="allowSynchronousContinuations">
    /// <see langword="true"/> if the <see cref="IAsyncEnumerable{T}"/> returned by <c>ReadAllAsync</c> may synchronously invoke continuations
    /// subscribed to notifications of pending async operations; <see langword="false"/> if all continuations
    /// should be invoked asynchronously.
    /// </param>
    /// <param name="ignoreLocalPublications">
    /// Whether to ignore messages sent by local publishers which live in the same node of the subscription.
    /// <para>
    /// This option may not be supported by some RMW implementations (e.g. rmw_fastrtps_cpp on foxy), and no error will be generated under such circumstances.
    /// If unsure, review the documentation of the RMW implementation you're using.
    /// </para>
    /// </param>
    /// <param name="contentFilter">
    /// The content filter to use when creating the subscription.
    /// <para>
    /// A <see cref="NotSupportedException"/> will be thrown when creating the subscription if the underlying RMW implementation does not support this feature.
    /// </para>
    /// <para>
    /// Supported distro(s): >=humble
    /// </para>
    /// </param>
    /// <param name="uniqueNetworkFlowEndpoints">
    /// Specifies the requirement on unique network flow endpoints.
    /// <para>
    /// Supported by: >= humble
    /// </para>
    /// </param>
    public SubscriptionOptions(
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        Action<LivelinessChangedEvent>? livelinessChangedHandler = null,
        Action<RequestedDeadlineMissedEvent>? requestedDeadlineMissedHandler = null,
        Action<IncompatibleQosEvent>? requestedQosIncompatibleHandler = null,
        bool allowSynchronousContinuations = false,
        bool ignoreLocalPublications = false,
        [SupportedSinceDistribution(RosEnvironment.Humble)]
        ContentFilterOptions? contentFilter = null,
        [SupportedSinceDistribution(RosEnvironment.Humble)]
        UniquenessRequirement uniqueNetworkFlowEndpoints = UniquenessRequirement.NotRequired)
    {
        if (fullMode == BoundedChannelFullMode.Wait)
        {
            throw new ArgumentException(
                "Setting fullMode to BoundedChannelFullMode.Wait is not supported " +
                "as it will cause the event loop to be blocked when the message delivery queue is full.",
                nameof(fullMode));
        }

        if (contentFilter != null)
        {
            RosEnvironment.Require(RosEnvironment.Humble, feature: "Content Filtered Topics");
        }

        if (uniqueNetworkFlowEndpoints != UniquenessRequirement.NotRequired)
        {
            RosEnvironment.Require(RosEnvironment.Humble, feature: "Network Flow Endpoints");
        }

        Qos = qos ?? QosProfile.Default;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        QueueSize = queueSize;
        FullMode = fullMode;
        LivelinessChangedHandler = livelinessChangedHandler;
        RequestedDeadlineMissedHandler = requestedDeadlineMissedHandler;
        RequestedQosIncompatibleHandler = requestedQosIncompatibleHandler;
        AllowSynchronousContinuations = allowSynchronousContinuations;
        IgnoreLocalPublications = ignoreLocalPublications;
        ContentFilter = contentFilter;
        UniqueNetworkFlowEndpoints = uniqueNetworkFlowEndpoints;
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
    /// This parameter is used for setting up the queue for devilering messages via <see cref="IAsyncEnumerable{T}"/>s returned by <c>ReadAllAsync</c>,
    /// and is irrelevant to <see cref="QosProfile.Depth"/>.
    /// </summary>
    /// <remarks>
    /// Setting to a small queue size may cause significant message drop under very high publishing rate,
    /// especially when <see cref="AllowSynchronousContinuations"/> is set to <see langword="false"/>.
    /// </remarks>
    public int QueueSize { get; }

    /// <summary>
    /// Behavior to use when the delivery queue is full.
    /// </summary>
    /// <remarks>
    /// <see cref="BoundedChannelFullMode.Wait"/> is not supported as it will cause the event loop to be blocked when the message delivery queue is full.
    /// </remarks>
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

    /// <summary>
    /// <see langword="true"/> if the <see cref="IAsyncEnumerable{T}"/> returned by <c>ReadAllAsync</c> may synchronously invoke continuations
    /// subscribed to notifications of pending async operations; <see langword="false"/> if all continuations
    /// should be invoked asynchronously.
    /// </summary>
    public bool AllowSynchronousContinuations { get; }

    /// <summary>
    /// Whether to ignore messages sent by local publishers which live in the same node of the subscription.
    /// </summary>
    /// <remarks>
    /// This option may not be supported by some RMW implementations (e.g. rmw_fastrtps_cpp on foxy), and no error will be generated under such circumstances.
    /// If unsure, review the documentation of the RMW implementation you're using.
    /// </remarks>
    public bool IgnoreLocalPublications { get; }

    /// <summary>
    /// The content filter to use when creating the subscription.
    /// </summary>
    /// <remarks>
    /// A <see cref="NotSupportedException"/> will be thrown when creating the subscription if the underlying RMW implementation does not support this feature.
    /// <para>
    /// Supported distro(s): >=humble
    /// </para>
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    public ContentFilterOptions? ContentFilter { get; }

    /// <summary>
    /// Specifies the requirement on unique network flow endpoints.
    /// </summary>
    /// <remarks>
    /// Supported by: >= humble
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    public UniquenessRequirement UniqueNetworkFlowEndpoints { get; }
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
    /// </para>
    /// </param>
    /// <param name="offeredDeadlineMissedHandler">
    /// Handler for receiving <see cref="OfferedDeadlineMissedEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="OfferedDeadlineMissedEvent"/>.
    /// </para>
    /// </param>
    /// <param name="offeredQosIncompatibleHandler">
    /// Handler for receiving <see cref="IncompatibleQosEvent"/>s.
    /// <para>
    /// Setting this option may cause <see cref="RclException"/> when creating the publisher,
    /// depending on whether the underlying RMW implementation supports <see cref="IncompatibleQosEvent"/>.
    /// </para>
    /// </param>
    /// <param name="uniqueNetworkFlowEndpoints">
    /// Specifies the requirement on unique network flow endpoints.
    /// <para>
    /// Supported by: >= humble
    /// </para>
    /// </param>
    public PublisherOptions(
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        Action<LivelinessLostEvent>? livelinessLostHandler = null,
        Action<OfferedDeadlineMissedEvent>? offeredDeadlineMissedHandler = null,
        Action<IncompatibleQosEvent>? offeredQosIncompatibleHandler = null,
        [SupportedSinceDistribution(RosEnvironment.Humble)]
        UniquenessRequirement uniqueNetworkFlowEndpoints = UniquenessRequirement.NotRequired)
    {
        if (uniqueNetworkFlowEndpoints != UniquenessRequirement.NotRequired)
        {
            RosEnvironment.Require(RosEnvironment.Humble, feature: "Network Flow Endpoints");
        }

        Qos = qos ?? QosProfile.Default;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        LivelinessLostHandler = livelinessLostHandler;
        OfferedDeadlineMissedHandler = offeredDeadlineMissedHandler;
        OfferedQosIncompatibleHandler = offeredQosIncompatibleHandler;
        UniqueNetworkFlowEndpoints = uniqueNetworkFlowEndpoints;
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

    /// <summary>
    /// Specifies the requirement on unique network flow endpoints.
    /// </summary>
    /// <remarks>
    /// Supported by: >= humble
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    public UniquenessRequirement UniqueNetworkFlowEndpoints { get; }
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