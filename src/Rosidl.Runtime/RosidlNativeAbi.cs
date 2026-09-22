namespace Rosidl.Runtime;

/// <summary>
/// Identifies a native ROSIDL message ABI.
/// </summary>
public enum RosidlNativeAbi
{
    /// <summary>
    /// The ROSIDL message ABI used by ROS 2 Foxy through Kilted.
    /// </summary>
    V1,

    /// <summary>
    /// The ROSIDL message ABI introduced by ROS 2 Lyrical.
    /// </summary>
    V2
}
