using Rosidl.Messages.Ros2csAbiTest;
using Rosidl.Runtime;

namespace Rcl.NET.Tests;

public class RosMessageBufferTests
{
    [Fact]
    public unsafe void AsRefAcceptsTheNativeAbi()
    {
        var buffer = new RosMessageBuffer(1, static (_, _) => { });

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            buffer.AsRef<Scalar.Priv>();
        }
        else
        {
            buffer.AsRef<Scalar.PrivV2>();
        }
    }

    [Fact]
    public unsafe void AsRefRejectsTheWrongNativeAbi()
    {
        var buffer = new RosMessageBuffer(1, static (_, _) => { });

        var exception = Assert.Throws<TypeInitializationException>(() =>
        {
            if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
            {
                buffer.AsRef<Scalar.PrivV2>();
            }
            else
            {
                buffer.AsRef<Scalar.Priv>();
            }
        });

        Assert.IsType<InvalidOperationException>(exception.InnerException);
    }

    [Fact]
    public unsafe void UnsafeAsRefDoesNotRequireNativeMetadata()
    {
        var value = 42;
        var buffer = new RosMessageBuffer((nint)(&value), static (_, _) => { });

        ref var actual = ref buffer.UnsafeAsRef<int>();

        Assert.Equal(42, actual);
    }
}
