@output(FloatSequence.g.cs, StructName=FloatSequence, ElementType=float, NativeStructName=float)@
@output(DoubleSequence.g.cs, StructName=DoubleSequence, ElementType=double, NativeStructName=double)@
@output(LongDoubleSequence.g.cs, StructName=LongDoubleSequence, ElementType=Bit128, NativeStructName=long_double)@
@output(CharSequence.g.cs, StructName=CharSequence, ElementType=sbyte, NativeStructName=char)@
@output(WCharSequence.g.cs, StructName=WCharSequence, ElementType=char, NativeStructName=wchar)@
@output(BooleanSequence.g.cs, StructName=BooleanSequence, ElementType=bool, NativeStructName=boolean)@
@output(OctetSequence.g.cs, StructName=OctetSequence, ElementType=byte, NativeStructName=octet)@
@output(UInt8Sequence.g.cs, StructName=UInt8Sequence, ElementType=byte, NativeStructName=uint8)@
@output(Int8Sequence.g.cs, StructName=Int8Sequence, ElementType=sbyte, NativeStructName=int8)@
@output(UInt16Sequence.g.cs, StructName=UInt16Sequence, ElementType=ushort, NativeStructName=uint16)@
@output(Int16Sequence.g.cs, StructName=Int16Sequence, ElementType=short, NativeStructName=int16)@
@output(UInt32Sequence.g.cs, StructName=UInt32Sequence, ElementType=uint, NativeStructName=uint32)@
@output(Int32Sequence.g.cs, StructName=Int32Sequence, ElementType=int, NativeStructName=int32)@
@output(UInt64Sequence.g.cs, StructName=UInt64Sequence, ElementType=ulong, NativeStructName=uint64)@
@output(Int64Sequence.g.cs, StructName=Int64Sequence, ElementType=long, NativeStructName=int64)@
@output(CStringSequence.g.cs, StructName=CStringSequence, ElementType=CString, NativeStructName=String)@
@output(U16StringSequence.g.cs, StructName=U16StringSequence, ElementType=U16String, NativeStructName=U16String)@

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rosidl.Runtime.Interop;

/// <summary>
/// Represents a rosidl_runtime_c__@NativeStructName@__Sequence.
/// </summary>
/// <remarks>
/// The <see cref="@StructName@"/> structure should be deallocated using <see cref="Dispose()"/> 
/// when it is no longer needed.
/// </remarks>
[StructLayout(LayoutKind.Sequential)]
public unsafe partial struct @StructName@ : IDisposable, IEquatable<@StructName@>
{
    private @ElementType@* _data;
    private nuint _size;
    private nuint _capacity;

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

    public override bool Equals(object obj) => obj is @StructName@ s ? Equals(s) : false;
    
    public override int GetHashCode() => HashCode.Combine((nint)_data, _size, _capacity);

    public static bool operator ==(@StructName@ lhs, @StructName@ rhs) => lhs.Equals(rhs);

    public static bool operator !=(@StructName@ lhs, @StructName@ rhs) => !(lhs == rhs);
    
    /// <summary>
    /// Check for <see cref="@StructName@"/> structure equality.
    /// </remarks>
    /// <returns>
    /// <see langword="true"/> if <see cref="@StructName@"/> structures are equal in size and content, otherwise <see langword="false"/>.
    /// </returns>
    public bool Equals(@StructName@ other) => AreEqual(in this, in other);

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
    public Span<@ElementType@> AsSpan() => new(_data, (int)_size);

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@StructName@"/> structure to copy from.</param>
    public void CopyFrom(@StructName@ src)
    {
        ThrowIfNonSuccess(TryCopy(in src, out this));
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@ElementType@"/> buffer to copy from.</param>
    public void CopyFrom(ReadOnlySpan<@ElementType@> src)
    {
        Finalize(ref this);
        ThrowIfNonSuccess(TryInitialize(src.Length, out this));
        src.CopyTo(AsSpan());
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="src"/>.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@StructName@"/> structure to copy from.</param>
    public void CopyFrom(@StructName@* value)
    {
        fixed (@StructName@* pThis = &this)
        {
            ThrowIfNonSuccess(TryCopy(value, pThis));
        }
    }

    private static bool TryInitialize(int size, out @StructName@ sequence)
    {
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
        return _PInvoke(input, output);

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__copy")]
        static extern bool _PInvoke(@StructName@* input, @StructName@* output);
    }

    private static bool AreEqual(in @StructName@ input, in @StructName@ output)
    {
        fixed (@StructName@* pInput = &input, pOutput = &output)
        {
            return _PInvoke(pInput, pOutput);
        }

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__are_equal")]
        static extern bool _PInvoke(@StructName@* lhs, @StructName@* rhs);
    }

    private static void Finalize(ref @StructName@ sequence)
    {
        fixed (@StructName@* pSequence = &sequence)
        {
            _PInvoke(pSequence);
        }

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__fini")]
        static extern void _PInvoke(@StructName@* sequence);
    }

    private static void ThrowIfNonSuccess(bool ret, [CallerMemberName]string caller = null)
    {
        if (!ret)
        {
            throw new RosidlException($"An error occurred when calling '@StructName@.{caller}'.");
        }
    }
}