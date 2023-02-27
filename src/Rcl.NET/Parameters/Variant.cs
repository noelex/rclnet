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

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.Integer"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(long value)
    {
        Kind = ValueKind.Integer;
        _integerValue = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.Integer"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(int value)
    {
        Kind = ValueKind.Integer;
        _integerValue = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.Bool"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(bool value)
    {
        Kind = ValueKind.Bool;
        _boolValue = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.Double"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(double value)
    {
        Kind = ValueKind.Double;
        _doubleValue = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.Double"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(float value)
    {
        Kind = ValueKind.Double;
        _doubleValue = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.String"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(string value)
    {
        Kind = ValueKind.String;
        _stringValue = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.BoolArray"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(bool[] value)
    {
        Kind = ValueKind.BoolArray;
        _boolArray = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.IntegerArray"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(long[] value)
    {
        Kind = ValueKind.IntegerArray;
        _integerArray = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.DoubleArray"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(double[] value)
    {
        Kind = ValueKind.DoubleArray;
        _doubleArray = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.StringArray"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
    public Variant(string[] value)
    {
        Kind = ValueKind.StringArray;
        _stringArray = value;
    }

    /// <summary>
    /// Create a new <see cref="Variant"/> of type <see cref="ValueKind.ByteArray"/>.
    /// </summary>
    /// <param name="value">The value of the <see cref="Variant"/>.</param>
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

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Boolean"/> value.
    /// </summary>
    /// <returns></returns>
    public bool AsBoolean()
    {
        CheckType(ValueKind.Bool);
        return _boolValue;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Int64"/> value.
    /// </summary>
    /// <returns></returns>
    public long AsInt64()
    {
        CheckType(ValueKind.Integer);
        return _integerValue;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Int32"/> value.
    /// </summary>
    /// <returns></returns>
    public int AsInt32()
    {
        CheckType(ValueKind.Integer);
        return (int)_integerValue;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Double"/> value.
    /// </summary>
    /// <returns></returns>
    public double AsDouble()
    {
        CheckType(ValueKind.Double);
        return _doubleValue;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Single"/> value.
    /// </summary>
    /// <returns></returns>
    public float AsSingle()
    {
        CheckType(ValueKind.Double);
        return (float)_doubleValue;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="String"/> value.
    /// </summary>
    /// <returns></returns>
    public string AsString()
    {
        CheckType(ValueKind.String);
        return _stringValue!;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Boolean"/>[] value.
    /// </summary>
    /// <returns></returns>
    public bool[] AsBooleanArray()
    {
        CheckType(ValueKind.BoolArray);
        return _boolArray!;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Byte"/>[] value.
    /// </summary>
    /// <returns></returns>
    public byte[] AsByteArray()
    {
        CheckType(ValueKind.ByteArray);
        return _byteArray!;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Int64"/>[] value.
    /// </summary>
    /// <returns></returns>
    public long[] AsInt64Array()
    {
        CheckType(ValueKind.IntegerArray);
        return _integerArray!;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="Double"/>[] value.
    /// </summary>
    /// <returns></returns>
    public double[] AsDoubleArray()
    {
        CheckType(ValueKind.DoubleArray);
        return _doubleArray!;
    }

    /// <summary>
    /// Reinterpret current <see cref="Variant"/> to a <see cref="String"/>[] value.
    /// </summary>
    /// <returns></returns>
    public string[] AsStringArray()
    {
        CheckType(ValueKind.StringArray);
        return _stringArray!;
    }

    /// <inheritdoc/>
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

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        return HashCode.Combine(Kind, _integerValue);
    }

    /// <inheritdoc/>
    public bool Equals(Variant other)
    {
        return other.Kind == Kind && _integerValue == other._integerValue;
    }

    /// <inheritdoc/>
    public override bool Equals(object? other)
    {
        if (other is Variant v) return Equals(v);
        return false;
    }

    /// <inheritdoc/>
    public static bool operator ==(Variant lhs, Variant rhs)
        => lhs.Equals(rhs);

    /// <inheritdoc/>
    public static bool operator !=(Variant lhs, Variant rhs)
        => !(lhs == rhs);

    /// <inheritdoc/>
    public static implicit operator Variant(bool value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(long value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(int value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(float value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(double value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(string value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(bool[] value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(byte[] value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(long[] value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(double[] value) => new(value);

    /// <inheritdoc/>
    public static implicit operator Variant(string[] value) => new(value);

    /// <inheritdoc/>
    public static explicit operator bool(Variant value) => value.AsBoolean();

    /// <inheritdoc/>
    public static explicit operator int(Variant value) => value.AsInt32();

    /// <inheritdoc/>
    public static explicit operator long(Variant value) => value.AsInt64();

    /// <inheritdoc/>
    public static explicit operator float(Variant value) => value.AsSingle();

    /// <inheritdoc/>
    public static explicit operator double(Variant value) => value.AsDouble();

    /// <inheritdoc/>
    public static explicit operator string(Variant value) => value.AsString();

    /// <inheritdoc/>
    public static explicit operator bool[](Variant value) => value.AsBooleanArray();

    /// <inheritdoc/>
    public static explicit operator byte[](Variant value) => value.AsByteArray();

    /// <inheritdoc/>
    public static explicit operator long[](Variant value) => value.AsInt64Array();

    /// <inheritdoc/>
    public static explicit operator double[](Variant value) => value.AsDoubleArray();

    /// <inheritdoc/>
    public static explicit operator string[](Variant value) => value.AsStringArray();
}
