using Rosidl.Runtime;
using System.Runtime.CompilerServices;

namespace Rcl;

/// <summary>
/// Represents an ROS message structure allocated in unmanaged memory.
/// </summary>
/// <remarks>
/// <see cref="Dispose"/> must be called before discarding references to the <see cref="RosMessageBuffer"/>, otherwise the message buffer will leak.
/// <para>
/// It's also the caller's responsibility to ensure that <see cref="Dispose"/> is called only once to avoid double-freeing the message buffer.
/// </para>
/// <para>
/// All public APIs involing <see cref="RosMessageBuffer"/> defines the contract of the ownership transfer of <see cref="RosMessageBuffer"/> instances.
/// Review their documentation carefully before use.
/// </para>
/// </remarks>
public readonly struct RosMessageBuffer : IDisposable
{
    private readonly object? _state;
    private readonly Action<nint, object?> _destroyCallback;

    public readonly nint Data;

    /// <summary>
    /// Create a new <see cref="RosMessageBuffer"/> with specified underlying message buffer.
    /// </summary>
    /// <param name="data">A pointer to the ROS message structure.</param>
    /// <param name="destroyCallback">A callback to be executed when <see cref="Dispose"/> is called to free the message buffer.</param>
    internal RosMessageBuffer(nint data, Action<nint, object?> destroyCallback, object? state = default)
    {
        Data = data;
        _destroyCallback = destroyCallback;
        _state = state;
    }

    /// <summary>
    /// Gets an empty <see cref="RosMessageBuffer"/>.
    /// </summary>
    public static readonly RosMessageBuffer Empty = new();

    /// <summary>
    /// Checks whether current <see cref="RosMessageBuffer"/> is empty.
    /// </summary>
    public bool IsEmpty => Data == nint.Zero;

    /// <summary>
    /// Access the containing message as a reference.
    /// </summary>
    /// <remarks>
    /// This method does not perform any check on the type of the underlying buffer.
    /// Calling this method with a mismatched message type parameter may cause
    /// unexpected behavior.
    /// </remarks>
    /// <typeparam name="T">Blittable structure definition of the message.</typeparam>
    /// <returns>A reference to the internal native data structure.</returns>
    public unsafe ref T AsRef<T>()
        where T : unmanaged
    {
        return ref Unsafe.AsRef<T>(Data.ToPointer());
    }

    /// <inheritdoc/>
    public void Dispose()
    {
        _destroyCallback(Data, _state);
    }

    /// <summary>
    /// Creates an <see cref="RosMessageBuffer"/> for specified message type.
    /// </summary>
    /// <typeparam name="T">Type of the message.</typeparam>
    /// <returns>A <see cref="RosMessageBuffer"/> containing the native data structure of the message.</returns>
    public static RosMessageBuffer Create<T>()
        where T : IMessage
    {
        return new(
             T.UnsafeCreate(), static (buffer, _) => T.UnsafeDestroy(buffer));
    }
}