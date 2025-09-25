namespace Rosidl.Runtime;

/// <summary>
/// Specifies that the class is an ROS interface providing dynamic type support.
/// </summary>
/// <remarks>
/// Creates a new <see cref="TypeSupportAttribute"/> with specified type support name.
/// </remarks>
/// <param name="name">The name of the type support.</param>
/// <param name="version">The source version of the type support.</param>
[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
public class TypeSupportAttribute(string name, string? version = null) : Attribute
{
    /// <summary>
    /// Gets the name of the type support.
    /// </summary>
    public string Name => name;

    /// <summary>
    /// Gets 
    /// </summary>
    public string? Version => version;
}
