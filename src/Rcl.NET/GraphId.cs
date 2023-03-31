using System.Runtime.InteropServices;

namespace Rcl;

/// <summary>
/// Represents a unique identifier for ROS communication entities.
/// </summary>
[StructLayout(LayoutKind.Explicit)]
public unsafe struct GraphId : IEquatable<GraphId>
{
    [FieldOffset(0)]
    private byte _data;

    [FieldOffset(0)]
    private long _data0;

    [FieldOffset(8)]
    private long _data1;

    [FieldOffset(16)]
    private long _data2;

    private Span<byte> AsSpan()
        => MemoryMarshal.CreateSpan(ref _data, 24);

    /// <summary>
    /// Create a new <see cref="GraphId"/> with its value copied from a <see cref="ReadOnlySpan{Byte}"/>.
    /// </summary>
    /// <param name="data">The <see cref="ReadOnlySpan{Byte}"/> to be copied from. The size of the <paramref name="data"/> must be either 16 or 24 bytes.</param>
    public unsafe GraphId(ReadOnlySpan<byte> data)
    {
        if (data.Length != 16 && data.Length != 24)
        {
            throw new ArgumentException($"Size of the GID data must be either 16 or 24 bytes.");
        }

        data.CopyTo(AsSpan());
    }

    /// <summary>
    /// Represents an uninitialized <see cref="GraphId"/>.
    /// </summary>
    public static readonly GraphId Empty = default;

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
        return HashCode.Combine(_data0, _data1, _data2);
    }

    /// <inheritdoc/>
    public static bool operator ==(GraphId a, GraphId b)
        => a.Equals(b);

    /// <inheritdoc/>
    public static bool operator !=(GraphId a, GraphId b)
        => !(a == b);

    /// <inheritdoc/>
    public bool Equals(GraphId other) =>
        _data0 == other._data0 &&
        _data1 == other._data1 &&
        _data2 == other._data2;

    /// <summary>
    /// Convert to <see cref="GraphId"/> to its string representation.
    /// </summary>
    /// <returns>A hexadecimal string representing the content of the <see cref="GraphId"/>.</returns>
    public override string ToString()
    {
        Span<char> output = stackalloc char[24 * 2 + 2];
        var data = AsSpan();

        var sep = 0;
        for (int i = 0; i < 24; i++)
        {
            data[i].TryFormat(output[(i * 2 + sep)..], out _, "X2");
            if (i == 7 || i == 15)
            {
                output[(i + 1) * 2 + sep] = '-';
                sep++;
            }
        }

        return output.ToString();
    }
}