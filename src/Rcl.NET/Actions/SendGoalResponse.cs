using Rosidl.Messages.Builtin;
using System.Runtime.InteropServices;

namespace Rcl.Actions;

[StructLayout(LayoutKind.Sequential)]
internal struct SendGoalResponse
{
    public bool Accepted;
    public Time.Priv Stamp;
}
