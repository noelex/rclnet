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
internal struct SendGoalResponseV2
{
    public bool Accepted;
    public Time.PrivV2 Stamp;
}

[StructLayout(LayoutKind.Sequential)]
internal struct GetResultRequest
{
    public UUID.Priv GoalId;
}

[StructLayout(LayoutKind.Sequential)]
internal struct GetResultRequestV2
{
    public UUID.PrivV2 GoalId;
}
