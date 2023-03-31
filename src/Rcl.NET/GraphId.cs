using System.Runtime.InteropServices;

namespace Rcl;

/// <summary>
/// Represents a unique identifier for ROS communication entities.
/// </summary>
public unsafe struct GraphId : IEquatable<GraphId>
{
    private fixed byte _data[24];

    private Span<byte> AsSpan()
    {
        fixed (void* p = _data)
        {
            return new(p, 24);
        }
    }

    /// <summary>
    /// Create a new <see cref="GraphId"/> with its value copied from a <see cref="ReadOnlySpan{Byte}"/>.
    /// </summary>
    /// <param name="data">The <see cref="ReadOnlySpan{Byte}"/> to be copied from. The size of the <paramref name="data"/> must be either 16 or 24 bytes.</param>
    public unsafe GraphId(ReadOnlySpan<byte> data)
    {
        if (data.Length != 16 || data.Length != 24)
        {
            throw new ArgumentException($"Size of the GID data must be either 16 or 24 bytes.");
        }

        data.CopyTo(AsSpan());
    }

    /// <summary>
    /// Create a new <see cref="GraphId"/> with its value copied from a <see cref="Guid"/>.
    /// </summary>
    /// <param name="guid">The <see cref="Guid"/> to be copied from.</param>
    public GraphId(Guid guid)
    {
        guid.TryWriteBytes(AsSpan());
    }

    /// <inheritdoc/>
    public override bool Equals(object? obj)
    {
        if (obj is GraphId other)
        {
            return Equals(other);
        }
        return false;
    }

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        var data = MemoryMarshal.Cast<byte, long>(AsSpan());
        return HashCode.Combine(data[0], data[1], data[2]);
    }

    /// <inheritdoc/>
    public static bool operator ==(GraphId a, GraphId b)
        => a.Equals(b);

    /// <inheritdoc/>
    public static bool operator !=(GraphId a, GraphId b)
        => !(a == b);

    /// <inheritdoc/>
    public bool Equals(GraphId other) => AsSpan().SequenceEqual(other.AsSpan());

    /// <summary>
    /// Convert to <see cref="GraphId"/> to its string representation.
    /// </summary>
    /// <returns>A hexadecimal string representing the content of the <see cref="GraphId"/>.</returns>
    public override string ToString()
    {
        Span<char> output = stackalloc char[24 * 2];
        for (int i = 0; i < 24; i++)
        {
            _data[i].TryFormat(output[(i * 2)..], out _, "x2");
        }

        return output.ToString();
    }
}