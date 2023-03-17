namespace Rcl.Actions;

/// <summary>
/// Represents an ROS action server.
/// </summary>
public interface IActionServer : IRclObject
{
    /// <summary>
    /// Gets the name of the action server.
    /// </summary>
    string Name { get; }
}
