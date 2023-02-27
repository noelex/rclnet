@output(CString.g.cs, StructName=CString, ElementType=SByte, NativeStructName=String)@
@output(U16String.g.cs, StructName=U16String, ElementType=Char, NativeStructName=U16String, Ext)@

using Microsoft.Toolkit.HighPerformance.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace Rosidl.Runtime.Interop;

/// <summary>
/// Represents a rosidl_runtime_c__@NativeStructName@.
/// </summary>
/// <remarks>
/// The contents of <see cref="@StructName@"/> are initialized to a single null character ('\0').
/// The string initially has size 0 and capacity 1.
/// All strings must be null-terminated.
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
    /// Initialize a <see cref="@StructName@"/> structure.
    /// </summary>
    public @StructName@()
    {
        ThrowIfNonSuccess(TryInitialize(out this));
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
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from a buffer containing the null terminated string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@ElementType@"/> buffer to copy from.</param>
    public @StructName@(@ElementType@* value)
    {
        CopyFrom(value);
    }

    /// <summary>
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from a buffer containing the string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@ElementType@"/> buffer to copy from.</param>
    /// <param name="n">Number of <see cref="@ElementType@"/>s to be copied.</param>
    public @StructName@(@ElementType@* value, int n)
    {
        CopyFrom(value, n);
    }

    /// <summary>
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from a buffer containing the string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@ElementType@"/> buffer to copy from.</param>
    public @StructName@(ReadOnlySpan<@ElementType@> value)
    {
        CopyFrom(value);
    }

@if Ext@
    /// <summary>
    /// Initialize a <see cref="@StructName@"/> structure, and copy its content from a buffer containing the string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="SByte"/> buffer to copy from.</param>
    /// <param name="n">Number of <see cref="SByte"/>s to be copied.</param>
    public @StructName@(sbyte* value, int n)
    {
        CopyFrom(value, n);
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from a buffer containing the string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="SByte"/> buffer to copy from.</param>
    /// <param name="n">Number of <see cref="SByte"/>s to be copied.</param>
    public void CopyFrom(sbyte* value, int n)
    {
        sbyte __empty = 0;
        if (value == null) value = &__empty;
        ThrowIfNonSuccess(TryAssignN(ref this, value, n));
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from a buffer containing the string.
    /// </summary>
    /// <param name="value">A <see cref="ReadOnlySpan{SByte}"/> buffer to copy from.</param>
    public void CopyFrom(ReadOnlySpan<sbyte> value)
    {
        fixed (sbyte* pValue = value)
        {
            CopyFrom(pValue, value.Length);
        }
    }

    /// <summary>
    /// Resize the internal storage of the the <see cref="@StructName@"/> structure to specified size.
    /// </summary>
    /// <param name="n">New size of the storage.</param>
    public void Resize(int n)
        => ThrowIfNonSuccess(TryResize(ref this, n));

    /// <inheritdoc/>
    public override string ToString() => StringMarshal.CreatePooledString(AsSpan());

@else@
    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from the specified Unicode character buffer with specified encoding.
    /// </summary>
    /// <param name="str">A <see cref="ReadOnlySpan{Char}"/> buffer to copy from.</param>
    /// <param name="encoding">Encoding to be used to encode the Unicode character buffer.</param>
    public void CopyFrom(ReadOnlySpan<char> str, Encoding encoding)
    {
        var bufferSize = encoding.GetMaxByteCount(str.Length);
        using var buffer = SpanOwner<byte>.Allocate(bufferSize);
        var count = encoding.GetBytes(str, buffer.Span);
        CopyFrom(MemoryMarshal.Cast<byte, sbyte>(buffer.Span.Slice(0, count)));
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from the specified Unicode character buffer with UTF-8 encoding.
    /// </summary>
    /// <param name="str">A <see cref="ReadOnlySpan{Char}"/> buffer to copy from.</param>
    public void CopyFrom(ReadOnlySpan<char> str)
        => CopyFrom(str, Encoding.UTF8);
    
    /// <inheritdoc/>
    public override string ToString() => StringMarshal.CreatePooledString(_data, Size);

@endif@
    /// <inheritdoc/>
    public override bool Equals(object obj) => obj is @StructName@ s ? Equals(s) : false;
    
    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine((nint)_data, _size, _capacity);

    /// <summary>
    /// Check for <see cref="@StructName@"/> structure equality.
    /// </summary>
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
    /// Gets the size of the contents of the string.
    /// </summary>
    public int Size => (int)_size;
   
    /// <summary>
    /// Gets the overall storage size of the string (counting the null terminator).
    /// </summary>
    public int Capacity => (int)_capacity;
    
    /// <summary>
    /// Creates a <see cref="Span{@ElementType@}"/> that represents the internal storage of the string.
    /// </summary>
    public Span<@ElementType@> AsSpan() => new(_data, (int)_size);

    /// <inheritdoc/>
    public static bool operator ==(@StructName@ lhs, @StructName@ rhs) => lhs.Equals(rhs);

    /// <inheritdoc/>
    public static bool operator !=(@StructName@ lhs, @StructName@ rhs) => !(lhs == rhs);
    
    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from <paramref name="src"/>.
    /// </summary>
    /// <param name="src">The source <see cref="@StructName@"/> structure to copy from.</param>
    /// <remarks>
    /// This functions performs a deep copy, as opposed to the shallow copy that plain assignment yields.
    /// </remarks>
    public void CopyFrom(@StructName@ src)
    {
        ThrowIfNonSuccess(TryCopy(in src, out this));
    }
    
    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from a buffer containing the null terminated string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@ElementType@"/> buffer to copy from.</param>
    public void CopyFrom(@ElementType@* value)
    {
        ThrowIfNonSuccess(TryAssign(ref this, value));
    }
    
    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from a buffer containing the string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@ElementType@"/> buffer to copy from.</param>
    /// <param name="n">Number of <see cref="@ElementType@"/>s to be copied.</param>
    public void CopyFrom(@ElementType@* value, int n)
    {
        @ElementType@ __empty = default;
        if (value == null) value = &__empty;
        ThrowIfNonSuccess(TryAssignN(ref this, value, n));
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from a buffer containing the string.
    /// </summary>
    /// <param name="value">A <see cref="ReadOnlySpan{@ElementType@}"/> buffer to copy from.</param>
    public void CopyFrom(ReadOnlySpan<@ElementType@> value)
    {
        fixed (@ElementType@* pValue = value)
        {
            CopyFrom(pValue, value.Length);
        }
    }

    /// <summary>
    /// Copy the content of the <see cref="@StructName@"/> structure from a buffer containing the string.
    /// </summary>
    /// <param name="value">A pointer to the <see cref="@StructName@"/> structure to copy from.</param>
    public void CopyFrom(@StructName@* value)
    {
        fixed (@StructName@* pThis = &this)
        {
            ThrowIfNonSuccess(TryCopy(value, pThis));
        }
    }

    private static bool TryInitialize(out @StructName@ str)
    {
        fixed (@StructName@* pStr = &str)
        {
            return _PInvoke(pStr);
        }
        
        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__init")]
        static extern bool _PInvoke(@StructName@* str);
    }

    private static void Finalize(ref @StructName@ str)
    {
        fixed (@StructName@* pStr = &str)
        {
            _PInvoke(pStr);
        }
        
        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__fini")]
        static extern void _PInvoke(@StructName@* str);
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
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__copy")]
        static extern bool _PInvoke(@StructName@* input, @StructName@* output);
    }

    private static bool AreEqual(in @StructName@ lhs, in @StructName@ rhs)
    {
        fixed (@StructName@* pLhs = &lhs, pRhs = &lhs)
        {
            return _PInvoke(pLhs, pRhs);
        }

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__are_equal")]
        static extern bool _PInvoke(@StructName@* lhs, @StructName@* rhs);
    }

    private static bool TryAssignN(ref @StructName@ str, @ElementType@* value, int n)
    {
        fixed (@StructName@* pStr = &str)
        {
            return _PInvoke(pStr, value, (uint)n);
        }

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__assignn")]
        static extern bool _PInvoke(@StructName@* str, @ElementType@* value, nuint n);
    }

    private static bool TryAssign(ref @StructName@ str, @ElementType@* value)
    {
        fixed (@StructName@* pStr = &str)
        {
            return _PInvoke(pStr, value);
        }
        
        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__assign")]
        static extern bool _PInvoke(@StructName@* str, @ElementType@* value);
    }

@if Ext@
    private static bool TryAssignN(ref @StructName@ str, sbyte* value, int n)
    {
        fixed (@StructName@* pStr = &str)
        {
            return _PInvoke(pStr, value, (uint)n);
        }

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__assignn_from_char")]
        static extern bool _PInvoke(@StructName@* str, sbyte* value, nuint n);
    }

    private static bool TryResize(ref @StructName@ str, int n)
    {
        fixed (@StructName@* pStr = &str)
        {
            return _PInvoke(pStr, (uint)n);
        }

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__resize")]
        static extern bool _PInvoke(@StructName@* str, nuint n);
    }
    
    /// <summary>
    /// Gets the length of the specified UTF-16 character sequence for which the first null char is determined.
    /// </summary>
    public static int GetLength(@ElementType@* data)
    {
        return (int)_PInvoke(data);

        [SuppressGCTransition]
        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__len")]
        static extern nuint _PInvoke(@ElementType@* value);
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