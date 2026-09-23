using Rosidl.Messages.Builtin;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Runtime.InteropServices;

namespace Rcl.Actions;

[StructLayout(LayoutKind.Sequential)]
[RosidlAbi(RosidlNativeAbi.V1)]
internal struct SendGoalResponse : IRosidlNative
{
    public static RosidlNativeAbi Abi => RosidlNativeAbi.V1;

    public bool Accepted;
    public Time.Priv Stamp;
}

[StructLayout(LayoutKind.Sequential)]
[RosidlAbi(RosidlNativeAbi.V2)]
internal struct SendGoalResponseV2 : IRosidlNative
{
    public static RosidlNativeAbi Abi => RosidlNativeAbi.V2;

    public bool Accepted;
    public Time.PrivV2 Stamp;
}

[StructLayout(LayoutKind.Sequential)]
[RosidlAbi(RosidlNativeAbi.V1)]
internal struct GetResultRequest : IRosidlNative
{
    public static RosidlNativeAbi Abi => RosidlNativeAbi.V1;

    public UUID.Priv GoalId;
}

[StructLayout(LayoutKind.Sequential)]
[RosidlAbi(RosidlNativeAbi.V2)]
internal struct GetResultRequestV2 : IRosidlNative
{
    public static RosidlNativeAbi Abi => RosidlNativeAbi.V2;

    public UUID.PrivV2 GoalId;
}
