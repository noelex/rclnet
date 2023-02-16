using Rcl.Qos;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Rcl;

public interface IRclSubscription : IRclObject
{
    QosProfile ActualQos { get; }

    bool IsValid { get; }

    int Publishers { get; }

    string? Name { get; }
}

public interface IRclSubscription<T> : IRclSubscription, IObservable<T>
    where T : IMessage
{
    IAsyncEnumerable<T> ReadAllAsync(CancellationToken cancellationToken = default);
}

public interface IRclNativeSubscription : IRclSubscription
{
    IAsyncEnumerable<RosMessageBuffer> ReadAllAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// Represents an ROS message structure allocated in unmanaged memory.
/// </summary>
/// <remarks>
/// <see cref="Dispose"/> must be called before discarding references to the <see cref="RosMessageBuffer"/>, otherwise the message buffer will leak.
/// <para>
/// It's also the caller's responsibility to ensure that <see cref="Dispose"/> is called only once to avoid double-freeing the message buffer.
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

    public static readonly RosMessageBuffer Empty = new();

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