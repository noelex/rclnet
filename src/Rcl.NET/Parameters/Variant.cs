using System.Runtime.InteropServices;

namespace Rcl.Parameters;

/// <summary>
/// Stores a value being one of the type defined in <see cref="ValueType"/>.
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
    public readonly ValueType Type;

    public Variant(long value)
    {
        Type = ValueType.Integer;
        _integerValue = value;
    }

    public Variant(int value)
    {
        Type = ValueType.Integer;
        _integerValue = value;
    }

    public Variant(bool value)
    {
        Type = ValueType.Bool;
        _boolValue = value;
    }

    public Variant(double value)
    {
        Type = ValueType.Double;
        _doubleValue = value;
    }

    public Variant(float value)
    {
        Type = ValueType.Double;
        _doubleValue = value;
    }

    public Variant(string value)
    {
        Type = ValueType.String;
        _stringValue = value;
    }

    public Variant(bool[] value)
    {
        Type = ValueType.BoolArray;
        _boolArray = value;
    }

    public Variant(long[] value)
    {
        Type = ValueType.IntegerArray;
        _integerArray = value;
    }

    public Variant(double[] value)
    {
        Type = ValueType.DoubleArray;
        _doubleArray = value;
    }

    public Variant(string[] value)
    {
        Type = ValueType.StringArray;
        _stringArray = value;
    }

    public Variant(byte[] value)
    {
        Type = ValueType.ByteArray;
        _byteArray = value;
    }

    private void CheckType(ValueType type)
    {
        if (Type != type)
        {
            throw new InvalidCastException();
        }
    }

    public bool AsBoolean()
    {
        CheckType(ValueType.Bool);
        return _boolValue;
    }

    public long AsInt64()
    {
        CheckType(ValueType.Integer);
        return _integerValue;
    }

    public int AsInt32()
    {
        CheckType(ValueType.Integer);
        return (int)_integerValue;
    }

    public double AsDouble()
    {
        CheckType(ValueType.Double);
        return _doubleValue;
    }

    public float AsSingle()
    {
        CheckType(ValueType.Double);
        return (float)_doubleValue;
    }

    public string AsString()
    {
        CheckType(ValueType.String);
        return _stringValue!;
    }

    public bool[] AsBooleanArray()
    {
        CheckType(ValueType.BoolArray);
        return _boolArray!;
    }

    public byte[] AsByteArray()
    {
        CheckType(ValueType.ByteArray);
        return _byteArray!;
    }

    public long[] AsInt64Array()
    {
        CheckType(ValueType.IntegerArray);
        return _integerArray!;
    }

    public double[] AsDoubleArray()
    {
        CheckType(ValueType.DoubleArray);
        return _doubleArray!;
    }

    public string[] AsStringArray()
    {
        CheckType(ValueType.StringArray);
        return _stringArray!;
    }

    public override string ToString()
    {
        return Type switch
        {
            ValueType.Bool => _boolValue.ToString(),
            ValueType.Double => _doubleValue.ToString(),
            ValueType.Integer => _integerValue.ToString(),
            ValueType.String => _stringValue ?? "",
            ValueType.BoolArray => "Count = " + _boolArray!.Length,
            ValueType.ByteArray => "Count = " + _byteArray!.Length,
            ValueType.IntegerArray => "Count = " + _integerArray!.Length,
            ValueType.StringArray => "Count = " + _stringArray!.Length,
            ValueType.DoubleArray => "Count = " + _doubleArray!.Length,
            _ => string.Empty,
        };
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Type, _integerValue);
    }

    public bool Equals(Variant other)
    {
        return other.Type == Type && _integerValue == other._integerValue;
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
