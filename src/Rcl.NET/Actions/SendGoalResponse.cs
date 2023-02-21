using Rosidl.Messages.Builtin;
using Rosidl.Messages.UniqueIdentifier;
using System.Runtime.InteropServices;

namespace Rcl.Actions;

[StructLayout(LayoutKind.Sequential)]
internal struct SendGoalResponse
{
    public bool Accepted;
    public Time.Priv Stamp;
}

[StructLayout(LayoutKind.Sequential)]
internal struct GetResultRequest
{
    public UUID.Priv GoalId;
}