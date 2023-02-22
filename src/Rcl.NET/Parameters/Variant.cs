using System.Runtime.InteropServices;

namespace Rcl.Parameters;

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

    [FieldOffset(16)]
    private readonly ParameterType _type;

    public Variant(long value)
    {
        _type = ParameterType.Integer;
        _integerValue = value;
    }

    public Variant(int value)
    {
        _type = ParameterType.Integer;
        _integerValue = value;
    }

    public Variant(bool value)
    {
        _type = ParameterType.Bool;
        _boolValue = value;
    }

    public Variant(double value)
    {
        _type = ParameterType.Double;
        _doubleValue = value;
    }

    public Variant(float value)
    {
        _type = ParameterType.Double;
        _doubleValue = value;
    }

    public Variant(string value)
    {
        _type = ParameterType.String;
        _stringValue = value;
    }

    public Variant(bool[] value)
    {
        _type = ParameterType.BoolArray;
        _boolArray = value;
    }

    public Variant(long[] value)
    {
        _type = ParameterType.IntegerArray;
        _integerArray = value;
    }

    public Variant(double[] value)
    {
        _type = ParameterType.DoubleArray;
        _doubleArray = value;
    }

    public Variant(string[] value)
    {
        _type = ParameterType.StringArray;
        _stringArray = value;
    }

    public Variant(byte[] value)
    {
        _type = ParameterType.ByteArray;
        _byteArray = value;
    }

    private void CheckType(ParameterType type)
    {
        if (_type != type)
        {
            throw new InvalidCastException();
        }
    }

    public bool AsBoolean()
    {
        CheckType(ParameterType.Bool);
        return _boolValue;
    }

    public long AsInt64()
    {
        CheckType(ParameterType.Integer);
        return _integerValue;
    }

    public int AsInt32()
    {
        CheckType(ParameterType.Integer);
        return (int)_integerValue;
    }

    public double AsDouble()
    {
        CheckType(ParameterType.Double);
        return _doubleValue;
    }

    public float AsSingle()
    {
        CheckType(ParameterType.Double);
        return (float)_doubleValue;
    }

    public string AsString()
    {
        CheckType(ParameterType.String);
        return _stringValue!;
    }

    public bool[] AsBooleanArray()
    {
        CheckType(ParameterType.BoolArray);
        return _boolArray!;
    }

    public byte[] AsByteArray()
    {
        CheckType(ParameterType.ByteArray);
        return _byteArray!;
    }

    public long[] AsInt64Array()
    {
        CheckType(ParameterType.IntegerArray);
        return _integerArray!;
    }

    public double[] AsDoubleArray()
    {
        CheckType(ParameterType.DoubleArray);
        return _doubleArray!;
    }

    public string[] AsStringArray()
    {
        CheckType(ParameterType.StringArray);
        return _stringArray!;
    }

    public override string ToString()
    {
        return _type switch
        {
            ParameterType.Bool => _boolValue.ToString(),
            ParameterType.Double => _doubleValue.ToString(),
            ParameterType.Integer => _integerValue.ToString(),
            ParameterType.String => _stringValue ?? "",
            ParameterType.BoolArray => "Count = " + _boolArray!.Length,
            ParameterType.ByteArray => "Count = " + _byteArray!.Length,
            ParameterType.IntegerArray => "Count = " + _integerArray!.Length,
            ParameterType.StringArray => "Count = " + _stringArray!.Length,
            ParameterType.DoubleArray => "Count = " + _doubleArray!.Length,
            _ => string.Empty,
        };
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(_type, _integerValue);
    }

    public bool Equals(Variant other)
    {
        return other._type == _type && _integerValue == other._integerValue;
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
