@output(FloatSequence.g.cs, StructName=FloatSequence, ElementType=Single, NativeStructName=float)@
@output(DoubleSequence.g.cs, StructName=DoubleSequence, ElementType=Double, NativeStructName=double)@
@output(LongDoubleSequence.g.cs, StructName=LongDoubleSequence, ElementType=Bit128, NativeStructName=long_double)@
@output(CharSequence.g.cs, StructName=CharSequence, ElementType=SByte, NativeStructName=char)@
@output(WCharSequence.g.cs, StructName=WCharSequence, ElementType=Char, NativeStructName=wchar)@
@output(BooleanSequence.g.cs, StructName=BooleanSequence, ElementType=Boolean, NativeStructName=boolean)@
@output(OctetSequence.g.cs, StructName=OctetSequence, ElementType=Byte, NativeStructName=octet)@
@output(UInt8Sequence.g.cs, StructName=UInt8Sequence, ElementType=Byte, NativeStructName=uint8)@
@output(Int8Sequence.g.cs, StructName=Int8Sequence, ElementType=SByte, NativeStructName=int8)@
@output(UInt16Sequence.g.cs, StructName=UInt16Sequence, ElementType=UInt16, NativeStructName=uint16)@
@output(Int16Sequence.g.cs, StructName=Int16Sequence, ElementType=Int16, NativeStructName=int16)@
@output(UInt32Sequence.g.cs, StructName=UInt32Sequence, ElementType=UInt32, NativeStructName=uint32)@
@output(Int32Sequence.g.cs, StructName=Int32Sequence, ElementType=Int32, NativeStructName=int32)@
@output(UInt64Sequence.g.cs, StructName=UInt64Sequence, ElementType=UInt64, NativeStructName=uint64)@
@output(Int64Sequence.g.cs, StructName=Int64Sequence, ElementType=Int64, NativeStructName=int64)@
@output(CStringSequence.g.cs, StructName=CStringSequence, ElementType=CString, NativeStructName=String)@
@output(U16StringSequence.g.cs, StructName=U16StringSequence, ElementType=U16String, NativeStructName=U16String)@
@output(FloatSequenceV2.g.cs, StructName=FloatSequenceV2, ElementType=Single, NativeStructName=float, V2)@
@output(DoubleSequenceV2.g.cs, StructName=DoubleSequenceV2, ElementType=Double, NativeStructName=double, V2)@
@output(LongDoubleSequenceV2.g.cs, StructName=LongDoubleSequenceV2, ElementType=Bit128, NativeStructName=long_double, V2)@
@output(CharSequenceV2.g.cs, StructName=CharSequenceV2, ElementType=SByte, NativeStructName=char, V2)@
@output(WCharSequenceV2.g.cs, StructName=WCharSequenceV2, ElementType=Char, NativeStructName=wchar, V2)@
@output(BooleanSequenceV2.g.cs, StructName=BooleanSequenceV2, ElementType=Boolean, NativeStructName=boolean, V2)@
@output(OctetSequenceV2.g.cs, StructName=OctetSequenceV2, ElementType=Byte, NativeStructName=octet, V2)@
@output(UInt8SequenceV2.g.cs, StructName=UInt8SequenceV2, ElementType=Byte, NativeStructName=uint8, V2)@
@output(Int8SequenceV2.g.cs, StructName=Int8SequenceV2, ElementType=SByte, NativeStructName=int8, V2)@
@output(UInt16SequenceV2.g.cs, StructName=UInt16SequenceV2, ElementType=UInt16, NativeStructName=uint16, V2)@
@output(Int16SequenceV2.g.cs, StructName=Int16SequenceV2, ElementType=Int16, NativeStructName=int16, V2)@
@output(UInt32SequenceV2.g.cs, StructName=UInt32SequenceV2, ElementType=UInt32, NativeStructName=uint32, V2)@
@output(Int32SequenceV2.g.cs, StructName=Int32SequenceV2, ElementType=Int32, NativeStructName=int32, V2)@
@output(UInt64SequenceV2.g.cs, StructName=UInt64SequenceV2, ElementType=UInt64, NativeStructName=uint64, V2)@
@output(Int64SequenceV2.g.cs, StructName=Int64SequenceV2, ElementType=Int64, NativeStructName=int64, V2)@
@output(CStringSequenceV2.g.cs, StructName=CStringSequenceV2, ElementType=CString, NativeStructName=String, V2)@
@output(U16StringSequenceV2.g.cs, StructName=U16StringSequenceV2, ElementType=U16String, NativeStructName=U16String, V2)@

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rosidl.Runtime.Interop;

/// <summary>
@if V2@
/// Represents a rosidl_runtime_c__@NativeStructName@__Sequence using <see cref="RosidlNativeAbi.V2"/>.
@else@
/// Represents a rosidl_runtime_c__@NativeStructName@__Sequence using <see cref="RosidlNativeAbi.V1"/>.
@endif@
/// </summary>
/// <remarks>
/// The <see cref="@StructName@"/> structure should be deallocated using <see cref="Dispose()"/>
/// when it is no longer needed.
/// </remarks>
@if V2@
[RosidlAbi(RosidlNativeAbi.V2)]
@else@
[RosidlAbi(RosidlNativeAbi.V1)]
@endif@
[StructLayout(LayoutKind.Sequential)]
public unsafe partial struct @StructName@ : IDisposable, IEquatable<@StructName@>, IRosidlNativeSequence<@ElementType@>
{
    private @ElementType@* _data;
    private nuint _size;
    private nuint _capacity;
@if V2@
    private byte _isRosidlBuffer;
    private byte _ownsRosidlBuffer;
@endif@

    /// <inheritdoc/>
@if V2@
    public static RosidlNativeAbi Abi => RosidlNativeAbi.V2;
@else@
    public static RosidlNativeAbi Abi => RosidlNativeAbi.V1;
@endif@

    /// <summary>
    /// Create a <see cref="@StructName@"/> structure with a specific size.
    /// </summary>
    /// <param name="size">Size of the internal storage of the <see cref="@StructName@"/> structure to be allocated.</param>
    /// <remarks>
    /// The <see cref="@StructName@"/> initially has size and capacity equal to the <paramref name="size"/> parameter.
    /// </remarks>
    public @StructName@(int size)
    {
        ThrowIfNonSuccess(TryInitialize(size, out this));
    }

    /// <summary>
    /// Create an empty <see cref="@StructName@"/> structure.
    /// </summary>
    public @StructName@()
    {
    }

    /// <summary>
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@StructName@"/> structure to copy from.</param>
    public @StructName@(@StructName@ src)
    {
        CopyFrom(src);
    }

    /// <summary>
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@StructName@"/> structure to copy from.</param>
    public @StructName@(@StructName@* src)
    {
        CopyFrom(src);
    }

    /// <summary>
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="ReadOnlySpan{@ElementType@}"/> to copy from.</param>
    public @StructName@(ReadOnlySpan<@ElementType@> src)
       :this(src.Length)
    {
        src.CopyTo(AsSpan());
    }

    /// <inheritdoc/>
    public override bool Equals(object obj) => obj is @StructName@ s ? Equals(s) : false;

    /// <inheritdoc/>
@if V2@
    public override int GetHashCode()
        => HashCode.Combine((nint)_data, _size, _capacity, _isRosidlBuffer, _ownsRosidlBuffer);
@else@
    public override int GetHashCode() => HashCode.Combine((nint)_data, _size, _capacity);
@endif@

    /// <inheritdoc/>
    public static bool operator ==(@StructName@ lhs, @StructName@ rhs) => lhs.Equals(rhs);

    /// <inheritdoc/>
    public static bool operator !=(@StructName@ lhs, @StructName@ rhs) => !(lhs == rhs);

    /// <summary>
    /// Check for <see cref="@StructName@"/> structure equality.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if <see cref="@StructName@"/> structures are equal in size and content, otherwise <see langword="false"/>.
    /// </returns>
@if V2@
    public bool Equals(@StructName@ other)
    {
        ThrowIfRosidlBuffer();
        other.ThrowIfRosidlBuffer();
        return AreEqual(in this, in other);
    }
@else@
    public bool Equals(@StructName@ other) => AreEqual(in this, in other);
@endif@

    /// <summary>
    /// Deallocate the memory of the <see cref="@StructName@"/> structure.
    /// </summary>
    /// <remarks>Calling the function with an already deallocated sequence is a no-op.</remarks>
    public void Dispose() => Finalize(ref this);

    /// <summary>
    /// Gets the size of the underlying <see cref="@ElementType@"/> buffer.
    /// </summary>
    public int Size => (int)_size;

    /// <summary>
    /// Creates a <see cref="Span{@ElementType@}"/> that represents the internal storage of the <see cref="@StructName@"/> structure.
    /// </summary>
@if V2@
    public Span<@ElementType@> AsSpan()
    {
        ThrowIfRosidlBuffer();
        return new(_data, (int)_size);
    }
@else@
    public Span<@ElementType@> AsSpan() => new(_data, (int)_size);
@endif@

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@StructName@"/> structure to copy from.</param>
    public void CopyFrom(@StructName@ src)
    {
@if V2@
        ThrowIfRosidlBuffer();
        src.ThrowIfRosidlBuffer();
@endif@
        ThrowIfNonSuccess(TryCopy(in src, out this));
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@ElementType@"/> buffer to copy from.</param>
    public void CopyFrom(ReadOnlySpan<@ElementType@> src)
    {
@if V2@
        ThrowIfRosidlBuffer();
@endif@
        Finalize(ref this);
        ThrowIfNonSuccess(TryInitialize(src.Length, out this));
        src.CopyTo(AsSpan());
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="value"/>.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@StructName@"/> structure to copy from.</param>
    public void CopyFrom(@StructName@* value)
    {
@if V2@
        ThrowIfRosidlBuffer();
        if (value != null)
        {
            value->ThrowIfRosidlBuffer();
        }
@endif@
        fixed (@StructName@* pThis = &this)
        {
            ThrowIfNonSuccess(TryCopy(value, pThis));
        }
    }

    private static bool TryInitialize(int size, out @StructName@ sequence)
    {
        RosidlRuntime.RequireNativeAbi(Abi);
        fixed (@StructName@* pSequence = &sequence)
        {
            return _PInvoke(pSequence, (uint)size);
        }

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__init")]
        static extern bool _PInvoke(@StructName@* sequence, nuint size);
    }

    private static bool TryCopy(in @StructName@ input, out @StructName@ output)
    {
        fixed (@StructName@* pInput = &input, pOutput = & output)
        {
            return TryCopy(pInput, pOutput);
        }
    }

    private static bool TryCopy(@StructName@* input, @StructName@* output)
    {
        RosidlRuntime.RequireNativeAbi(Abi);
        return _PInvoke(input, output);

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__copy")]
        static extern bool _PInvoke(@StructName@* input, @StructName@* output);
    }

    private static bool AreEqual(in @StructName@ input, in @StructName@ output)
    {
        RosidlRuntime.RequireNativeAbi(Abi);
        fixed (@StructName@* pInput = &input, pOutput = &output)
        {
            return _PInvoke(pInput, pOutput);
        }

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__are_equal")]
        static extern bool _PInvoke(@StructName@* lhs, @StructName@* rhs);
    }

    private static void Finalize(ref @StructName@ sequence)
    {
        RosidlRuntime.RequireNativeAbi(Abi);
        fixed (@StructName@* pSequence = &sequence)
        {
            _PInvoke(pSequence);
        }

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__fini")]
        static extern void _PInvoke(@StructName@* sequence);
    }

@if V2@
    private readonly void ThrowIfRosidlBuffer()
    {
        if (_isRosidlBuffer != 0)
        {
            throw new NotSupportedException("rosidl::Buffer-backed sequences are not supported.");
        }
    }

@endif@
    private static void ThrowIfNonSuccess(bool ret, [CallerMemberName]string caller = null)
    {
        if (!ret)
        {
            throw new RosidlException($"An error occurred when calling '@StructName@.{caller}'.");
        }
    }
}
