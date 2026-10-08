using Rosidl.Runtime;
using System.Runtime.CompilerServices;

namespace Rcl;

/// <summary>
/// Represents an ROS message structure in unmanaged memory, including middleware-owned loans.
/// </summary>
/// <remarks>
/// The owner of a non-empty buffer must call <see cref="Dispose"/> exactly once when the buffer is no longer needed,
/// unless responsibility for releasing it has been transferred to another owner.
/// Borrowed buffers must not be disposed by the borrower.
/// <para>
/// Copies of this structure share the same native memory; copying does not create a new message or owner.
/// After disposing a buffer or transferring its release responsibility, the previous holder must not access or dispose
/// any of its copies or native references.
/// </para>
/// <para>
/// Review each API's ownership and lifetime contract before using a buffer.
/// </para>
/// <para>
/// For a loaned buffer, the middleware owns the memory and the loan holder is responsible for returning it.
/// Call <see cref="Dispose"/> to return the loan.
/// The originating publisher or subscription and its context must remain alive until the loan is returned
/// or transferred. Returning or transferring a loan invalidates every buffer copy and native reference.
/// </para>
/// </remarks>
public readonly struct RosMessageBuffer : IDisposable
{
    private readonly object? _state;
    private readonly Action<nint, object?> _destroyCallback;

    /// <summary>
    /// A pointer to the buffer storing the native message structure.
    /// </summary>
    /// <remarks>The pointer is borrowed from the buffer and must not outlive its memory or borrowing scope.</remarks>
    public readonly nint Data;

    /// <summary>
    /// Create a new <see cref="RosMessageBuffer"/> with specified underlying message buffer.
    /// </summary>
    /// <param name="data">A pointer to the ROS message structure.</param>
    /// <param name="destroyCallback">A callback to be executed when <see cref="Dispose"/> is called to free the message buffer.</param>
    /// <param name="state">A custom state object to be passed to the <paramref name="destroyCallback"/>.</param>
    internal RosMessageBuffer(nint data, Action<nint, object?> destroyCallback, object? state = default)
    {
        Data = data;
        _destroyCallback = destroyCallback;
        _state = state;
    }

    /// <summary>
    /// Gets an empty <see cref="RosMessageBuffer"/>.
    /// </summary>
    /// <remarks>An empty buffer owns no memory and can be disposed safely.</remarks>
    public static readonly RosMessageBuffer Empty = new();

    /// <summary>
    /// Checks whether current <see cref="RosMessageBuffer"/> is empty.
    /// </summary>
    public bool IsEmpty => Data == nint.Zero;

    /// <summary>
    /// Access the containing message as a reference.
    /// </summary>
    /// <remarks>
    /// This method verifies that <typeparamref name="T"/> uses the native ABI of the
    /// current ROS distribution. It does not verify the message type of the underlying buffer.
    /// The returned reference is valid only while the buffer is live and any borrowing scope remains active.
    /// It does not extend the buffer's lifetime or transfer ownership.
    /// </remarks>
    /// <typeparam name="T">Native structure definition of the message.</typeparam>
    /// <returns>A reference to the internal native data structure.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe ref T AsRef<T>()
        where T : unmanaged, IRosidlNative
    {
        RosidlRuntime.RequireNativeAbi(T.Abi);
        return ref Unsafe.AsRef<T>(Data.ToPointer());
    }

    /// <summary>
    /// Access the containing message as a reference without validating its native ABI.
    /// </summary>
    /// <remarks>
    /// This method does not perform any check on the type or ABI of the underlying buffer.
    /// Calling this method with a mismatched type parameter may cause unexpected behavior.
    /// The returned reference is valid only while the buffer is live and any borrowing scope remains active.
    /// It does not extend the buffer's lifetime or transfer ownership.
    /// </remarks>
    /// <typeparam name="T">Blittable structure definition of the message.</typeparam>
    /// <returns>A reference to the internal native data structure.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe ref T UnsafeAsRef<T>()
        where T : unmanaged
    {
        return ref Unsafe.AsRef<T>(Data.ToPointer());
    }

    /// <summary>
    /// Releases an owned buffer or returns a held middleware loan. Does nothing for an empty buffer.
    /// </summary>
    /// <remarks>
    /// Non-empty buffers must be disposed exactly once across all copies, and only by their owner or loan holder.
    /// This method does not make subsequent disposal or access to a non-empty buffer safe.
    /// </remarks>
    public void Dispose()
    {
        if (!IsEmpty)
        {
            _destroyCallback(Data, _state);
        }
    }

    /// <summary>
    /// Creates an <see cref="RosMessageBuffer"/> for specified message type.
    /// </summary>
    /// <typeparam name="T">Type of the message.</typeparam>
    /// <returns>A <see cref="RosMessageBuffer"/> containing the native data structure of the message.</returns>
    /// <remarks>The caller owns the returned buffer and must dispose it exactly once when no longer needed.</remarks>
    public static RosMessageBuffer Create<T>()
        where T : IMessage
    {
        return new(
             T.UnsafeCreate(), static (buffer, _) => T.UnsafeDestroy(buffer));
    }
}
