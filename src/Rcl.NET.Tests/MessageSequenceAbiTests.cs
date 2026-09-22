using Rcl.Introspection;
using Rosidl.Messages.Rcl;
using System.Runtime.CompilerServices;

namespace Rcl.NET.Tests;

public class MessageSequenceAbiTests
{
    [SkippableFact]
    public void LyricalParameterEventLayoutMatchesNativeIntrospection()
    {
        Skip.If(!RosEnvironment.IsLyrical);

        var native = MessageIntrospection.Create(ParameterEvent.GetTypeSupportHandle());
        var message = default(ParameterEvent.PrivV2);

        Assert.Equal(native.SizeOf, Unsafe.SizeOf<ParameterEvent.PrivV2>());
        Assert.Equal(native.GetMemberOffset(2), OffsetOf(ref message, ref message.NewParameters));
        Assert.Equal(native.GetMemberOffset(3), OffsetOf(ref message, ref message.ChangedParameters));
        Assert.Equal(native.GetMemberOffset(4), OffsetOf(ref message, ref message.DeletedParameters));
    }

    private static int OffsetOf<T, TField>(ref T message, ref TField field)
        => checked((int)Unsafe.ByteOffset(ref Unsafe.As<T, byte>(ref message), ref Unsafe.As<TField, byte>(ref field)));
}
