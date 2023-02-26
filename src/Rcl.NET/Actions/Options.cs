using Rcl.Qos;
using System.Text;

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
    public ActionClientOptions(
        QosProfile? goalServiceQos = null,
        QosProfile? resultServiceQos = null,
        QosProfile? cancelServiceQos = null,
        QosProfile? feedbackTopicQos = null,
        QosProfile? statusTopicQos = null,
        Encoding? textEncoding = null)
    {
        GoalServiceQos = goalServiceQos ?? QosProfile.ServicesDefault;
        ResultServiceQos = resultServiceQos ?? QosProfile.ServicesDefault;
        CancelServiceQos = cancelServiceQos ?? QosProfile.ServicesDefault;
        FeedbackTopicQos = feedbackTopicQos ??
            (RosEnvironment.IsFoxy
                ? QosProfile.SensorData : QosProfile.Default);
        StatusTopicQos = statusTopicQos ?? QosProfile.ActionStatusDefault;
        TextEncoding = textEncoding ?? Encoding.UTF8;
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