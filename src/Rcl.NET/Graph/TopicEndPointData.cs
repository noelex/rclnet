using Rcl.Qos;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.Graph;

record struct NameWithType(string Name, string Type);

record struct TopicEndPointData(
    GraphId Id,
    string Type,
    NodeName Node,
    QosProfile QosProfile);

internal struct GraphId : IEquatable<GraphId>
{
    // These fields are initialized by directly copying the struct.
#pragma warning disable CS0649
    private readonly long _d1, _d2, _d3;
#pragma warning restore CS0649

    public GraphId(ReadOnlySpan<byte> data)
    {
        this = MemoryMarshal.Cast<byte, GraphId>(data)[0];
    }

    /// <summary>
    /// This is for iron or later, as the size of the GID is changed from 24 to 16 bytes.
    /// </summary>
    /// <param name="guid"></param>
    public unsafe GraphId(Guid guid)
    {
        Unsafe.AsRef<Guid>(Unsafe.AsPointer(ref this)) = guid;
    }

    public unsafe GraphId(byte* data)
        : this(new ReadOnlySpan<byte>(data, sizeof(GraphId)))
    { }

    public override bool Equals(object? obj)
    {
        if (obj is GraphId other)
        {
            return Equals(other);
        }
        return false;
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(_d1, _d2, _d3);
    }

    public static bool operator ==(GraphId a, GraphId b)
        => a.Equals(b);

    public static bool operator !=(GraphId a, GraphId b)
        => !(a == b);

    public bool Equals(GraphId other)
        => _d1 == other._d1 && _d2 == other._d2 && _d3 == other._d3;
}