namespace Rosidl.Runtime;

/// <summary>
/// Identifies the native ROSIDL ABI used by an ABI-sensitive structure.
/// </summary>
[AttributeUsage(AttributeTargets.Struct, AllowMultiple = false, Inherited = false)]
public sealed class RosidlAbiAttribute : Attribute
{
    /// <summary>
    /// Creates an attribute for the specified native ROSIDL ABI.
    /// </summary>
    /// <param name="abi">The native ROSIDL ABI used by the annotated structure.</param>
    public RosidlAbiAttribute(RosidlNativeAbi abi)
    {
        Abi = abi;
    }

    /// <summary>
    /// Gets the native ROSIDL ABI used by the annotated structure.
    /// </summary>
    public RosidlNativeAbi Abi { get; }
}
