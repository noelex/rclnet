using Rcl.Interop;
using Rosidl.Runtime.Interop;

namespace Rcl.Graph;

/// <summary>
/// Represents the name of a ROS node.
/// </summary>
/// <param name="Name">The name of the node.</param>
/// <param name="Namespace">The namespace of the node.</param>
public record struct NodeName(string Name, string Namespace = "/")
{
    /// <summary>
    /// Gets the fully-qualified name of the node.
    /// </summary>
    public string FullyQualifiedName
    {
        get
        {
            // This creates a pooled string with up to one heap allocation.
            // If the string is already pooled, this will be allocation-free.
            Span<char> buffer = stackalloc char[Name.Length + Namespace.Length];
            Namespace.CopyTo(buffer);
            Name.CopyTo(buffer.Slice(Namespace.Length));
            return StringMarshal.CreatePooledString(buffer);
        }
    }

    public override string ToString() => FullyQualifiedName;
}