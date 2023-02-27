namespace Rcl.Graph;

/// <summary>
/// Delegate for handling <see cref="RosGraphEvent"/>s.
/// </summary>
/// <param name="args"></param>
public delegate void GraphChangedEventHandler(RosGraphEvent args);

public partial class RosGraph
{
    /// <summary>
    /// Listen to the event which will be triggered when ROS graph is changed.
    /// </summary>
    public event GraphChangedEventHandler? GraphChanged;
}