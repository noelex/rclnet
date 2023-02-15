using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime.Interop;

/// <summary>
/// Provides a uniform way to reinterpret 128-bit binary data as <see cref="System.Int128"/> or <see cref="System.UInt128"/>.
/// </summary>
[StructLayout(LayoutKind.Explicit)]
public struct Bit128 : IEquatable<Bit128>
{
    [FieldOffset(0)]
    public Int128 Int128;

    [FieldOffset(0)]
    public UInt128 UInt128;

    public bool Equals(Bit128 other)
    {
        return Int128 == other.Int128;
    }

    public override bool Equals([NotNullWhen(true)] object? obj)
    {
        if (obj is Bit128 other) return Equals(other);
        return false;
    }

    public override int GetHashCode()
    {
        return Int128.GetHashCode();
    }

    public static bool operator ==(Bit128 left, Bit128 right)
        => left.Equals(right);

    public static bool operator !=(Bit128 left, Bit128 right)
        => !(left == right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static implicit operator Bit128(Int128 v) => new() { Int128 = v };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static implicit operator Bit128(UInt128 v) => new() { UInt128 = v };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Int128(Bit128 v) => v.Int128;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator UInt128(Bit128 v) => v.UInt128;
}