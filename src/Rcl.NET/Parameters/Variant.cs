using System.Runtime.InteropServices;

namespace Rcl.Parameters;

/// <summary>
/// Stores a value being one of the type defined in <see cref="ValueKind"/>.
/// </summary>
[StructLayout(LayoutKind.Explicit)]
public readonly struct Variant : IEquatable<Variant>
{
    [FieldOffset(0)]
    private readonly long _integerValue;

    [FieldOffset(0)]
    private readonly double _doubleValue;

    [FieldOffset(0)]
    private readonly bool _boolValue;

    [FieldOffset(8)]
    private readonly string? _stringValue;

    [FieldOffset(8)]
    private readonly byte[]? _byteArray;

    [FieldOffset(8)]
    private readonly bool[]? _boolArray;

    [FieldOffset(8)]
    private readonly long[]? _integerArray;

    [FieldOffset(8)]
    private readonly double[]? _doubleArray;

    [FieldOffset(8)]
    private readonly string[]? _stringArray;

    /// <summary>
    /// Type of the value store in the <see cref="Variant"/>.
    /// </summary>
    [FieldOffset(16)]
    public readonly ValueKind Kind;

    public Variant(long value)
    {
        Kind = ValueKind.Integer;
        _integerValue = value;
    }

    public Variant(int value)
    {
        Kind = ValueKind.Integer;
        _integerValue = value;
    }

    public Variant(bool value)
    {
        Kind = ValueKind.Bool;
        _boolValue = value;
    }

    public Variant(double value)
    {
        Kind = ValueKind.Double;
        _doubleValue = value;
    }

    public Variant(float value)
    {
        Kind = ValueKind.Double;
        _doubleValue = value;
    }

    public Variant(string value)
    {
        Kind = ValueKind.String;
        _stringValue = value;
    }

    public Variant(bool[] value)
    {
        Kind = ValueKind.BoolArray;
        _boolArray = value;
    }

    public Variant(long[] value)
    {
        Kind = ValueKind.IntegerArray;
        _integerArray = value;
    }

    public Variant(double[] value)
    {
        Kind = ValueKind.DoubleArray;
        _doubleArray = value;
    }

    public Variant(string[] value)
    {
        Kind = ValueKind.StringArray;
        _stringArray = value;
    }

    public Variant(byte[] value)
    {
        Kind = ValueKind.ByteArray;
        _byteArray = value;
    }

    private void CheckType(ValueKind type)
    {
        if (Kind != type)
        {
            throw new InvalidCastException();
        }
    }

    public bool AsBoolean()
    {
        CheckType(ValueKind.Bool);
        return _boolValue;
    }

    public long AsInt64()
    {
        CheckType(ValueKind.Integer);
        return _integerValue;
    }

    public int AsInt32()
    {
        CheckType(ValueKind.Integer);
        return (int)_integerValue;
    }

    public double AsDouble()
    {
        CheckType(ValueKind.Double);
        return _doubleValue;
    }

    public float AsSingle()
    {
        CheckType(ValueKind.Double);
        return (float)_doubleValue;
    }

    public string AsString()
    {
        CheckType(ValueKind.String);
        return _stringValue!;
    }

    public bool[] AsBooleanArray()
    {
        CheckType(ValueKind.BoolArray);
        return _boolArray!;
    }

    public byte[] AsByteArray()
    {
        CheckType(ValueKind.ByteArray);
        return _byteArray!;
    }

    public long[] AsInt64Array()
    {
        CheckType(ValueKind.IntegerArray);
        return _integerArray!;
    }

    public double[] AsDoubleArray()
    {
        CheckType(ValueKind.DoubleArray);
        return _doubleArray!;
    }

    public string[] AsStringArray()
    {
        CheckType(ValueKind.StringArray);
        return _stringArray!;
    }

    public override string ToString()
    {
        return Kind switch
        {
            ValueKind.Bool => _boolValue.ToString(),
            ValueKind.Double => _doubleValue.ToString(),
            ValueKind.Integer => _integerValue.ToString(),
            ValueKind.String => _stringValue ?? "",
            ValueKind.BoolArray => "Count = " + _boolArray!.Length,
            ValueKind.ByteArray => "Count = " + _byteArray!.Length,
            ValueKind.IntegerArray => "Count = " + _integerArray!.Length,
            ValueKind.StringArray => "Count = " + _stringArray!.Length,
            ValueKind.DoubleArray => "Count = " + _doubleArray!.Length,
            _ => string.Empty,
        };
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Kind, _integerValue);
    }

    public bool Equals(Variant other)
    {
        return other.Kind == Kind && _integerValue == other._integerValue;
    }

    public override bool Equals(object? other)
    {
        if (other is Variant v) return Equals(v);
        return false;
    }

    public static bool operator ==(Variant lhs, Variant rhs)
        => lhs.Equals(rhs);

    public static bool operator !=(Variant lhs, Variant rhs)
        => !(lhs == rhs);

    public static implicit operator Variant(bool value) => new(value);

    public static implicit operator Variant(long value) => new(value);

    public static implicit operator Variant(int value) => new(value);

    public static implicit operator Variant(float value) => new(value);

    public static implicit operator Variant(double value) => new(value);

    public static implicit operator Variant(string value) => new(value);

    public static implicit operator Variant(bool[] value) => new(value);

    public static implicit operator Variant(byte[] value) => new(value);

    public static implicit operator Variant(long[] value) => new(value);

    public static implicit operator Variant(double[] value) => new(value);

    public static implicit operator Variant(string[] value) => new(value);

    public static explicit operator bool(Variant value) => value.AsBoolean();

    public static explicit operator int(Variant value) => value.AsInt32();

    public static explicit operator long(Variant value) => value.AsInt64();

    public static explicit operator float(Variant value) => value.AsSingle();

    public static explicit operator double(Variant value) => value.AsDouble();

    public static explicit operator string(Variant value) => value.AsString();

    public static explicit operator bool[](Variant value) => value.AsBooleanArray();

    public static explicit operator byte[](Variant value) => value.AsByteArray();

    public static explicit operator long[](Variant value) => value.AsInt64Array();

    public static explicit operator double[](Variant value) => value.AsDoubleArray();

    public static explicit operator string[](Variant value) => value.AsStringArray();
}
