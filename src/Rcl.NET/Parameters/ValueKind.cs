namespace Rcl.Parameters;

/// <summary>
/// Represents the type of the value stored in <see cref="Variant"/>.
/// </summary>
public enum ValueKind : byte
{
    /// <summary>
    /// Value type of the <see cref="Variant"/> is unknown.
    /// </summary>
    Unknown = 0,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="Boolean"/>.
    /// </summary>
    Bool = 1,

    /// <summary>
    /// The <see cref="Variant"/> stores an <see cref="Int64"/>.
    /// </summary>
    Integer = 2,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="System.Double"/>.
    /// </summary>
    Double = 3,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="System.String"/>.
    /// </summary>
    String = 4,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="Byte"/>[].
    /// </summary>
    ByteArray = 5,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="Boolean"/>[].
    /// </summary>
    BoolArray = 6,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="Int64"/>[].
    /// </summary>
    IntegerArray = 7,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="System.Double"/>[].
    /// </summary>
    DoubleArray = 8,

    /// <summary>
    /// The <see cref="Variant"/> stores a <see cref="System.String"/>[].
    /// </summary>
    StringArray = 9,
}
