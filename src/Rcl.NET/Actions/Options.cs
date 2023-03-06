using Rcl.Qos;
using System.Text;
using System.Threading.Channels;

namespace Rcl.Actions;

/// <summary>
/// Extra options for creating action clients.
/// </summary>
public record ActionClientOptions
{
    /// <summary>
    /// Create a new <see cref="ActionClientOptions"/>.
    /// </summary>
    /// <param name="goalServiceQos">
    /// The QoS settings to be used for goal service.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="resultServiceQos">
    /// The QoS settings to be used for result service.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="cancelServiceQos">
    /// The QoS settings to be used for cancel service.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="feedbackTopicQos">
    /// The QoS settings to be used for feedback topic.
    /// <para>
    /// Defaults to <see cref="QosProfile.SensorData"/> on foxy,
    /// <see cref="QosProfile.Default"/> on other distributions.
    /// </para>
    /// </param>
    /// <param name="statusTopicQos">
    /// The QoS settings to be used for status topic.
    /// <para>
    /// Defaults to <see cref="QosProfile.ActionStatusDefault"/>.
    /// </para>
    /// </param>
    /// <param name="textEncoding">
    /// Encoding of the string values in the message.
    /// <para>This setting only applies to messages received/sent as POCOs, not native message buffers.</para>
    /// </param>
    /// <param name="queueSize">
    /// Capacity of the feedback message delivery queue.
    /// This parameter is used for setting up the queue for devilering messages via <see cref="IAsyncEnumerable{T}"/>s returned by <c>ReadFeedbacksAsync</c>,
    /// and is irrelevant to <see cref="QosProfile.Depth"/>.
    /// <para>
    /// Setting to a small queue size may cause significant message drop under very high publishing rate,
    /// especially when <paramref name="allowSynchronousContinuation"/> is set to <see langword="false"/>.
    /// </para>
    /// </param>
    /// <param name="fullMode">
    /// Behavior to use when the feedback message delivery queue is full.
    /// <para>
    /// <see cref="BoundedChannelFullMode.Wait"/> is not supported as it will cause the event loop to be blocked when the message delivery queue is full.
    /// </para>
    /// </param>
    /// <param name="allowSynchronousContinuation">
    /// <see langword="true"/> if the <see cref="IAsyncEnumerable{T}"/> returned by <c>ReadFeedbacksAsync</c> may synchronously invoke continuations
    /// subscribed to notifications of pending async operations; <see langword="false"/> if all continuations
    /// should be invoked asynchronously.
    /// </param>
    public ActionClientOptions(
        QosProfile? goalServiceQos = null,
        QosProfile? resultServiceQos = null,
        QosProfile? cancelServiceQos = null,
        QosProfile? feedbackTopicQos = null,
        QosProfile? statusTopicQos = null,
        Encoding? textEncoding = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        bool allowSynchronousContinuation = false)
    {
        if (fullMode == BoundedChannelFullMode.Wait)
        {
            throw new ArgumentException(
                "Setting fullMode to BoundedChannelFullMode.Wait is not supported " +
                "as it will cause the event loop to be blocked when the message delivery queue is full.",
                nameof(fullMode));
        }

        GoalServiceQos = goalServiceQos ?? QosProfile.ServicesDefault;
        ResultServiceQos = resultServiceQos ?? QosProfile.ServicesDefault;
        CancelServiceQos = cancelServiceQos ?? QosProfile.ServicesDefault;
        FeedbackTopicQos = feedbackTopicQos ??
            (RosEnvironment.IsFoxy
                ? QosProfile.SensorData : QosProfile.Default);
        StatusTopicQos = statusTopicQos ?? QosProfile.ActionStatusDefault;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        FullMode = fullMode;
        QueueSize = queueSize;
        AllowSynchronousContinuations = allowSynchronousContinuation;
    }

    /// <summary>
    /// Gets a <see cref="ActionClientOptions"/> instance with with all options set to default values.
    /// </summary>
    public static ActionClientOptions Default { get; } = new();

    /// <summary>
    /// The QoS settings to be used for goal service.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile GoalServiceQos { get; }

    /// <summary>
    /// The QoS settings to be used for result service.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile ResultServiceQos { get; }

    /// <summary>
    /// The QoS settings to be used for cancel service.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile CancelServiceQos { get; }

    /// <summary>
    /// The QoS settings to be used for feedback topic.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.SensorData"/> on foxy,
    /// <see cref="QosProfile.Default"/> on other distributions.
    /// </remarks>
    public QosProfile FeedbackTopicQos { get; }

    /// <summary>
    /// The QoS settings to be used for status topic.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ActionStatusDefault"/>.
    /// </remarks>
    public QosProfile StatusTopicQos { get; }

    /// <summary>
    /// Encoding of the string values in the message.
    /// </summary>
    /// <remarks>
    /// This setting only applies to messages received/sent as POCOs, not native message buffers.
    /// </remarks>
    public Encoding TextEncoding { get; }

    /// <summary>
    /// Capacity of the feedback message delivery queue.
    /// This parameter is used for setting up the queue for devilering messages via <see cref="IAsyncEnumerable{T}"/>s returned by <c>ReadFeedbacksAsync</c>,
    /// and is irrelevant to <see cref="QosProfile.Depth"/>.
    /// </summary>
    /// <remarks>
    /// Setting to a small queue size may cause significant message drop under very high publishing rate,
    /// especially when <paramref name="allowSynchronousContinuation"/> is set to <see langword="false"/>.
    /// </remarks>
    public int QueueSize { get; }

    /// <summary>
    /// Behavior to use when the feeback message delivery queue is full.
    /// </summary>
    /// <remarks>
    /// <see cref="BoundedChannelFullMode.Wait"/> is not supported as it will cause the event loop to be blocked when the message delivery queue is full.
    /// </remarks>
    public BoundedChannelFullMode FullMode { get; }

    /// <summary>
    /// <see langword="true"/> if the <see cref="IAsyncEnumerable{T}"/> returned by <c>ReadFeedbacksAsync</c> may synchronously invoke continuations
    /// subscribed to notifications of pending async operations; <see langword="false"/> if all continuations
    /// should be invoked asynchronously.
    /// </summary>
    public bool AllowSynchronousContinuations { get; }
}

/// <summary>
/// Extra options for creating action servers.
/// </summary>
public record ActionServerOptions
{
    /// <summary>
    /// Create a new <see cref="ActionServerOptions"/>.
    /// </summary>
    /// <param name="goalServiceQos">
    /// The QoS settings to be used for goal service.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="resultServiceQos">
    /// The QoS settings to be used for result service.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="cancelServiceQos">
    /// The QoS settings to be used for cancel service.
    /// <para>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </para>
    /// </param>
    /// <param name="feedbackTopicQos">
    /// The QoS settings to be used for feedback topic.
    /// <para>
    /// Defaults to <see cref="QosProfile.SensorData"/> on foxy,
    /// <see cref="QosProfile.Default"/> on other distributions.
    /// </para>
    /// </param>
    /// <param name="statusTopicQos">
    /// The QoS settings to be used for status topic.
    /// <para>
    /// Defaults to <see cref="QosProfile.ActionStatusDefault"/>.
    /// </para>
    /// </param>
    /// <param name="textEncoding">
    /// Encoding of the string values in the message.
    /// <para>This setting only applies to messages received/sent as POCOs, not native message buffers.</para>
    /// </param>
    /// <param name="resultTimeout">
    /// Timeout of the result cache.
    /// Setting to a negative <see cref="TimeSpan"/> will cause the results to be kept indefinitely until server shutdown.
    /// Setting to <see cref="TimeSpan.Zero"/> will disable caching and the result is removed as soon as the action
    /// client gets the result.
    /// <para>Default is 15 minutes.</para>
    /// </param>
    public ActionServerOptions(
        QosProfile? goalServiceQos = null,
        QosProfile? resultServiceQos = null,
        QosProfile? cancelServiceQos = null,
        QosProfile? feedbackTopicQos = null,
        QosProfile? statusTopicQos = null,
        Encoding? textEncoding = null,
        TimeSpan? resultTimeout = null)
    {
        GoalServiceQos = goalServiceQos ?? QosProfile.ServicesDefault;
        ResultServiceQos = resultServiceQos ?? QosProfile.ServicesDefault;
        CancelServiceQos = cancelServiceQos ?? QosProfile.ServicesDefault;
        FeedbackTopicQos = feedbackTopicQos ??
            (RosEnvironment.IsFoxy
                ? QosProfile.SensorData : QosProfile.Default);
        StatusTopicQos = statusTopicQos ?? QosProfile.ActionStatusDefault;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        ResultTimeout = resultTimeout ?? TimeSpan.FromMinutes(15);
    }

    /// <summary>
    /// Gets a <see cref="ActionServerOptions"/> instance with with all options set to default values.
    /// </summary>
    public static ActionServerOptions Default { get; } = new();

    /// <summary>
    /// The QoS settings to be used for goal service.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile GoalServiceQos { get; }

    /// <summary>
    /// The QoS settings to be used for result service.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile ResultServiceQos { get; }

    /// <summary>
    /// The QoS settings to be used for cancel service.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ServicesDefault"/>.
    /// </remarks>
    public QosProfile CancelServiceQos { get; }

    /// <summary>
    /// The QoS settings to be used for feedback topic.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.SensorData"/> on foxy,
    /// <see cref="QosProfile.Default"/> on other distributions.
    /// </remarks>
    public QosProfile FeedbackTopicQos { get; }

    /// <summary>
    /// The QoS settings to be used for status topic.
    /// </summary>
    /// <remarks>
    /// Defaults to <see cref="QosProfile.ActionStatusDefault"/>.
    /// </remarks>
    public QosProfile StatusTopicQos { get; }

    /// <summary>
    /// Encoding of the string values in the message.
    /// </summary>
    /// <remarks>
    /// This setting only applies to messages received/sent as POCOs, not native message buffers.
    /// </remarks>
    public Encoding TextEncoding { get; }

    /// <summary>
    /// Timeout of the result cache.
    /// Setting to a negative <see cref="TimeSpan"/> will cause the results to be kept indefinitely until server shutdown.
    /// Setting to <see cref="TimeSpan.Zero"/> will disable caching and the result is removed as soon as the action
    /// client gets the result.
    /// </summary>
    /// <remarks>
    /// Default is 15 minutes.
    /// </remarks>
    public TimeSpan ResultTimeout { get; }
}