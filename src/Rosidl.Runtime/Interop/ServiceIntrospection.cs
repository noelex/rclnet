using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime.Interop;

[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct ServiceMembers
{
    public readonly sbyte* ServiceNamespace;
    public readonly sbyte* ServiceName;
    public readonly MessageMembers* RequestMembers;
    public readonly MessageMembers* ResponseMembers;
}
