using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime;

/// <summary>
/// Specifies that the class is an ROS interface providing dynamic type support.
/// </summary>
[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
public class TypeSupportAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="TypeSupportAttribute"/> with specified type support name.
    /// </summary>
    /// <param name="name">The name of the type support.</param>
    public TypeSupportAttribute(string name)
    {
        Name = name;
    }

    /// <summary>
    /// Gets the name of the type support.
    /// </summary>
    public string Name { get; set; }
}
