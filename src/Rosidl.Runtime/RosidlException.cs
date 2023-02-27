namespace Rosidl.Runtime;

/// <summary>
/// Represents errors occur in ROS IDL message processing.
/// </summary>
public class RosidlException : Exception
{
    /// <summary>
    /// Create a new <see cref="RosidlException"/> with specified message.
    /// </summary>
    /// <param name="message">The error message.</param>
    public RosidlException(string message) : base(message) { }
}
