using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime;

[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
public class TypeSupportAttribute : Attribute
{
    public TypeSupportAttribute(string name)
    {
        Name = name;
    }

    public string Name { get; set; }
}
