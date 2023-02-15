namespace Rcl.Graph;

public delegate void GraphChangedEventHandler(RosGraphEvent args);

public partial class RosGraph
{
    /// <summary>
    /// Listen to the event which will be triggered when ROS graph is changed.
    /// </summary>
    public event GraphChangedEventHandler? GraphChanged;
}