using Rcl.Qos;

namespace Rcl;

/// <summary>
/// QoS Liveliness Lost information provided by a publisher.
/// </summary>
/// <param name="TotalCount">
/// Lifetime cumulative number of times that a previously-alive Publisher became not alive due to
/// a failure to actively signal its liveliness within its offered liveliness period.
/// This count does not change when an already not alive Publisher simply remains not alive for
/// another liveliness period.
/// </param>
/// <param name="Delta">
/// The change in <see cref="TotalCount"/> since the last time the status was last read.
/// </param>
public record struct LivelinessLostEvent(int TotalCount, int Delta);


/// <summary>
/// QoS Requested Deadline Missed information provided by a subscription.
/// </summary>
/// <param name="TotalCount">
/// Lifetime cumulative number of missed deadlines detected for any instance read by the
/// subscription.
/// Missed deadlines accumulate; that is, each deadline period the total_count will be incremented
/// by one for each instance for which data was not received.
/// </param>
/// <param name="Delta">
/// The incremental number of deadlines detected since the status was read.
/// </param>
public record struct RequestedDeadlineMissedEvent(int TotalCount, int Delta);

/// <summary>
/// QoS Liveliness Changed information provided by a subscription.
/// </summary>
/// <param name="AliveCount">
/// The total number of currently active Publishers which publish to the topic associated with
/// the Subscription.
/// This count increases when a newly matched Publisher asserts its liveliness for the first time
/// or when a Publisher previously considered to be not alive reasserts its liveliness.
/// The count decreases when a Publisher considered alive fails to assert its liveliness and
/// becomes not alive, whether because it was deleted normally or for some other reason.
/// </param>
/// <param name="NotAliveCount">
/// The total count of current Publishers which publish to the topic associated with the
/// Subscription that are no longer asserting their liveliness.
/// This count increases when a Publisher considered alive fails to assert its liveliness and
/// becomes not alive for some reason other than the normal deletion of that Publisher.
/// It decreases when a previously not alive Publisher either reasserts its liveliness or is
/// deleted normally.
/// </param>
/// <param name="AliveCountDelta">
/// The change in the <see cref="AliveCount"/> since the status was last read.
/// </param>
/// <param name="NotAliveCountDelta">
/// The change in the <see cref="NotAliveCount"/> since the status was last read.
/// </param>
public record struct LivelinessChangedEvent(
    int AliveCount,
    int NotAliveCount,
    int AliveCountDelta,
    int NotAliveCountDelta
);

/// <summary>
/// QoS Deadline Missed information provided by a publisher.
/// </summary>
/// <param name="TotalCount">
/// Lifetime cumulative number of offered deadline periods elapsed during which a Publisher failed
/// to provide data.
/// Missed deadlines accumulate; that is, each deadline period the <see cref="TotalCount"/> will be incremented
/// by one.
/// </param>
/// <param name="Delta">The change in <see cref="TotalCount"/> since the last time the status was last read.</param>
public record struct OfferedDeadlineMissedEvent(int TotalCount, int Delta);

/// <summary>
/// Requested / Offered QoS Incompatible information.
/// </summary>
/// <param name="TotalCount">
/// Total cumulative number of times the concerned subscription / publisher discovered a
/// publisher / subscription for the same topic with an offered / requested QoS that was incompatible
/// with that requested / offered by the subscription / publisher.
/// </param>
/// <param name="Delta">The change in <see cref="TotalCount"/> since the last time the status was read.</param>
/// <param name="LastPolicyKind">
/// The Qos Policy Kind of one of the policies that was found to be
/// incompatible the last time an incompatibility was detected.
/// </param>
public record struct IncompatibleQosEvent(int TotalCount, int Delta, QosPolicyKind LastPolicyKind);

