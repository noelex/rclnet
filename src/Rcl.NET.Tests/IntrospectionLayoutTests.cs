using Rcl.Introspection;
using System.Runtime.CompilerServices;
using static Rcl.Interop.RclCommon;

namespace Rcl.NET.Tests;

public class IntrospectionLayoutTests
{
    [Fact]
    public void LyricalMessageMemberAppendsRosidlBufferFlag()
    {
        var member = default(MessageMember_Lyrical);

        Assert.Equal(
            Unsafe.SizeOf<MessageMember_Jazzy>(),
            OffsetOf(in member, in member.IsRosidlBuffer));
        Assert.Equal(
            Unsafe.SizeOf<MessageMember_Jazzy>() + IntPtr.Size,
            Unsafe.SizeOf<MessageMember_Lyrical>());
    }

    [Fact]
    public void LyricalEndpointTypesPreserveNativeValues()
    {
        Assert.Equal(3, (int)rmw_endpoint_type_t.RMW_ENDPOINT_CLIENT);
        Assert.Equal(4, (int)rmw_endpoint_type_t.RMW_ENDPOINT_SERVER);
    }

    private static int OffsetOf<T, TField>(in T value, in TField field)
    {
        ref var start = ref Unsafe.As<T, byte>(ref Unsafe.AsRef(in value));
        ref var target = ref Unsafe.As<TField, byte>(ref Unsafe.AsRef(in field));
        return checked((int)Unsafe.ByteOffset(ref start, ref target));
    }
}
