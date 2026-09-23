using Rcl.Introspection;
using Rosidl.Messages.Rcl;
using Rosidl.Runtime;
using System.Runtime.CompilerServices;

namespace Rcl.NET.Tests;

public class MessageSequenceAbiTests
{
    [Fact]
    public void ParameterEventLayoutMatchesNativeIntrospection()
    {
        var native = MessageIntrospection.Create(ParameterEvent.GetTypeSupportHandle());
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            var message = default(ParameterEvent.Priv);
            AssertLayout(
                native,
                Unsafe.SizeOf<ParameterEvent.Priv>(),
                OffsetOf(ref message, ref message.NewParameters),
                OffsetOf(ref message, ref message.ChangedParameters),
                OffsetOf(ref message, ref message.DeletedParameters));
        }
        else
        {
            var message = default(ParameterEvent.PrivV2);
            AssertLayout(
                native,
                Unsafe.SizeOf<ParameterEvent.PrivV2>(),
                OffsetOf(ref message, ref message.NewParameters),
                OffsetOf(ref message, ref message.ChangedParameters),
                OffsetOf(ref message, ref message.DeletedParameters));
        }
    }

    private static void AssertLayout(
        IMessageIntrospection native,
        int size,
        int newParametersOffset,
        int changedParametersOffset,
        int deletedParametersOffset)
    {
        Assert.Equal(native.SizeOf, size);
        Assert.Equal(native.GetMemberOffset(2), newParametersOffset);
        Assert.Equal(native.GetMemberOffset(3), changedParametersOffset);
        Assert.Equal(native.GetMemberOffset(4), deletedParametersOffset);
    }

    private static int OffsetOf<T, TField>(ref T message, ref TField field)
        => checked((int)Unsafe.ByteOffset(ref Unsafe.As<T, byte>(ref message), ref Unsafe.As<TField, byte>(ref field)));
}
