namespace Rcl.Actions;

/// <summary>
/// Represents an ROS action server.
/// </summary>
/// <remarks>
/// Disposal stops accepting work and signals cancellation of active goals without waiting for their handlers to finish.
/// Buffers borrowed by an active handler remain valid until its execution task completes.
/// </remarks>
public interface IActionServer : IRclObject
{
    /// <summary>
    /// Gets the name of the action server.
    /// </summary>
    string Name { get; }
}
