using Rosidl.Runtime.Interop;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Xunit;

namespace Rosidl.Runtime.Tests;

public class SequenceLayoutTests
{
    [Fact]
    public void SequenceLayoutsMatchNativeAbis()
    {
        AssertSequenceLayout<FloatSequence, FloatSequenceV2>();
        AssertSequenceLayout<DoubleSequence, DoubleSequenceV2>();
        AssertSequenceLayout<LongDoubleSequence, LongDoubleSequenceV2>();
        AssertSequenceLayout<CharSequence, CharSequenceV2>();
        AssertSequenceLayout<WCharSequence, WCharSequenceV2>();
        AssertSequenceLayout<BooleanSequence, BooleanSequenceV2>();
        AssertSequenceLayout<OctetSequence, OctetSequenceV2>();
        AssertSequenceLayout<UInt8Sequence, UInt8SequenceV2>();
        AssertSequenceLayout<Int8Sequence, Int8SequenceV2>();
        AssertSequenceLayout<UInt16Sequence, UInt16SequenceV2>();
        AssertSequenceLayout<Int16Sequence, Int16SequenceV2>();
        AssertSequenceLayout<UInt32Sequence, UInt32SequenceV2>();
        AssertSequenceLayout<Int32Sequence, Int32SequenceV2>();
        AssertSequenceLayout<UInt64Sequence, UInt64SequenceV2>();
        AssertSequenceLayout<Int64Sequence, Int64SequenceV2>();
        AssertSequenceLayout<CStringSequence, CStringSequenceV2>();
        AssertSequenceLayout<U16StringSequence, U16StringSequenceV2>();
    }

    [Fact]
    public void BufferBackedUInt8SequenceRejectsDataAccess()
    {
        var source = CreateBufferBacked<UInt8SequenceV2>();
        var target = default(UInt8SequenceV2);

        AssertBufferNotSupported(() => { source.AsSpan(); });
        AssertBufferNotSupported(() => { _ = new UInt8SequenceV2(source); });
        AssertBufferNotSupported(() => target.CopyFrom(source));
        AssertBufferNotSupported(() => source.CopyFrom(target));
        AssertBufferNotSupported(() => source.CopyFrom(ReadOnlySpan<byte>.Empty));
        AssertBufferNotSupported(() => source.Equals(target));
        AssertBufferNotSupported(() => target.Equals(source));
        AssertBufferNotSupported(() => CopyFromPointer(source));
    }

    [Fact]
    public void NonUInt8V2SequencesIgnoreOpaqueBufferFlags()
    {
        var octet = CreateBufferBacked<OctetSequenceV2>();
        var cString = CreateBufferBacked<CStringSequenceV2>();
        var u16String = CreateBufferBacked<U16StringSequenceV2>();

        Assert.True(octet.AsSpan().IsEmpty);
        Assert.True(cString.AsSpan().IsEmpty);
        Assert.True(u16String.AsSpan().IsEmpty);
        Assert.Equal(default(OctetSequenceV2).GetHashCode(), octet.GetHashCode());
        Assert.Equal(default(CStringSequenceV2).GetHashCode(), cString.GetHashCode());
        Assert.Equal(default(U16StringSequenceV2).GetHashCode(), u16String.GetHashCode());
    }

    [Fact]
    public unsafe void V2StringSequencesCreatedByNativeCodeCanBeAccessed()
    {
        if (RosidlRuntime.NativeAbi != RosidlNativeAbi.V2)
        {
            return;
        }

        var cString = CStringSequenceV2.Create(1);
        var u16String = U16StringSequenceV2.Create(1);
        try
        {
            Assert.True(cString != null);
            Assert.True(u16String != null);
            Assert.Equal(1, cString->AsSpan().Length);
            Assert.Equal(1, u16String->AsSpan().Length);

            SetOpaqueBufferFlags(ref *cString);
            SetOpaqueBufferFlags(ref *u16String);

            Assert.Equal(1, cString->AsSpan().Length);
            Assert.Equal(1, u16String->AsSpan().Length);

            var cStringCopy = new CStringSequenceV2(*cString);
            var u16StringCopy = new U16StringSequenceV2(*u16String);
            try
            {
                Assert.True(cString->Equals(cStringCopy));
                Assert.True(u16String->Equals(u16StringCopy));
            }
            finally
            {
                cStringCopy.Dispose();
                u16StringCopy.Dispose();
            }

            cString->CopyFrom(ReadOnlySpan<CString>.Empty);
            u16String->CopyFrom(ReadOnlySpan<U16String>.Empty);
            Assert.True(cString->AsSpan().IsEmpty);
            Assert.True(u16String->AsSpan().IsEmpty);
        }
        finally
        {
            CStringSequenceV2.Destroy(cString);
            U16StringSequenceV2.Destroy(u16String);
        }
    }

    [Fact]
    public void V2StringSequencesExposeNativeLifecycleMethods()
    {
        Assert.NotNull(typeof(CStringSequenceV2).GetMethod(nameof(CStringSequenceV2.Create)));
        Assert.NotNull(typeof(CStringSequenceV2).GetMethod(nameof(CStringSequenceV2.Destroy)));
        Assert.NotNull(typeof(U16StringSequenceV2).GetMethod(nameof(U16StringSequenceV2.Create)));
        Assert.NotNull(typeof(U16StringSequenceV2).GetMethod(nameof(U16StringSequenceV2.Destroy)));
    }

    [Fact]
    public void NativeSequencesExposeGenericContracts()
    {
        AssertSequenceContract<UInt8Sequence, byte>(RosidlNativeAbi.V1);
        AssertSequenceContract<UInt8SequenceV2, byte>(RosidlNativeAbi.V2);
        AssertSequenceContract<CStringSequence, CString>(RosidlNativeAbi.V1);
        AssertSequenceContract<CStringSequenceV2, CString>(RosidlNativeAbi.V2);
    }

    private static void AssertSequenceLayout<TV1, TV2>()
        where TV1 : unmanaged, IRosidlNative
        where TV2 : unmanaged, IRosidlNative
    {
        var flagsOffset = 3 * IntPtr.Size;

        Assert.Equal(flagsOffset, Unsafe.SizeOf<TV1>());
        Assert.Equal(4 * IntPtr.Size, Unsafe.SizeOf<TV2>());
        Assert.Equal(flagsOffset, Marshal.OffsetOf<TV2>("_isRosidlBuffer").ToInt32());
        Assert.Equal(flagsOffset + 1, Marshal.OffsetOf<TV2>("_ownsRosidlBuffer").ToInt32());
        Assert.Equal(RosidlNativeAbi.V1, TV1.Abi);
        Assert.Equal(RosidlNativeAbi.V2, TV2.Abi);
        Assert.Equal(RosidlNativeAbi.V1, typeof(TV1).GetCustomAttribute<RosidlAbiAttribute>()?.Abi);
        Assert.Equal(RosidlNativeAbi.V2, typeof(TV2).GetCustomAttribute<RosidlAbiAttribute>()?.Abi);
    }

    private static void AssertSequenceContract<TSequence, T>(RosidlNativeAbi expectedAbi)
        where TSequence : unmanaged, IRosidlNativeSequence<T>
        where T : unmanaged
    {
        var sequence = default(TSequence);

        Assert.Equal(expectedAbi, TSequence.Abi);
        Assert.Equal(0, sequence.Size);
        Assert.True(sequence.AsSpan().IsEmpty);
    }

    private static T CreateBufferBacked<T>() where T : unmanaged
    {
        var value = default(T);
        SetOpaqueBufferFlags(ref value);
        return value;
    }

    private static void SetOpaqueBufferFlags<T>(ref T value) where T : unmanaged
    {
        var bytes = MemoryMarshal.AsBytes(MemoryMarshal.CreateSpan(ref value, 1));
        bytes[3 * IntPtr.Size] = 1;
        bytes[3 * IntPtr.Size + 1] = 1;
    }

    private static unsafe void CopyFromPointer(UInt8SequenceV2 source)
    {
        var target = default(UInt8SequenceV2);
        target.CopyFrom(&source);
    }

    private static void AssertBufferNotSupported(Action action)
    {
        var exception = Assert.Throws<NotSupportedException>(action);
        Assert.Equal("rosidl::Buffer-backed sequences are not supported.", exception.Message);
    }
}
