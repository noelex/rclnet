namespace Rosidl.Generator.CSharp;

public enum PrimitiveTypes
{
    Bool,
    Int8,
    Int16,
    Int32,
    Int64,
    UInt8,
    UInt16,
    UInt32,
    UInt64,
    Float32,
    Float64,
    String,
    WString
}

public record MessageIdentifier(string Package, string? SubFolder, string Name)
{
    public override string ToString()
    {
        return SubFolder == null ? $"{Package}/{Name}" : $"{Package}/{SubFolder}/{Name}";
    }
}

public abstract record TypeMetadata;

public record PrimitiveTypeMetadata(PrimitiveTypes ValueType, int? MaxLength) : TypeMetadata
{
    public override string ToString() => MaxLength is null ? ValueType.ToString().ToLower() : $"{ValueType.ToString().ToLower()}<={MaxLength}";
}

public record ArrayTypeMetadata(
    TypeMetadata ElementType,
    int? Length,
    bool IsUpperBounded
) : TypeMetadata
{
    public override string ToString() => IsUpperBounded ? $"{ElementType}[<={Length}]" : $"{ElementType}[{Length}]";
}

public record ComplexTypeMetadata(MessageIdentifier Id) : TypeMetadata
{
    public string? Version { get; internal set; }

    public override string ToString()
    {
        return Id.ToString();
    }
}

public record MessageMetadata(
    MessageIdentifier Id,
    string[] Comments,
    IReadOnlyCollection<FieldMetadata> Fields
) : ComplexTypeMetadata(Id)
{
    public override string ToString()
    {
        return base.ToString();
    }

    public ComplexTypeMetadata Ref() => new(Id);
}

public record ServiceMetadata(
    MessageIdentifier Id,
    string[] Comments,
    IReadOnlyCollection<FieldMetadata> RequestFields,
    IReadOnlyCollection<FieldMetadata> ResponseFields,
    IReadOnlyCollection<FieldMetadata> EventFields
) : ComplexTypeMetadata(Id)
{
    public override string ToString()
    {
        return base.ToString();
    }
}

public record ActionMetadata(
    MessageIdentifier Id,
    string[] Comments,
    IReadOnlyCollection<FieldMetadata> GoalFields,
    IReadOnlyCollection<FieldMetadata> FeedbackFields,
    IReadOnlyCollection<FieldMetadata> ResultFields
) : ComplexTypeMetadata(Id)
{
    public override string ToString()
    {
        return base.ToString();
    }
}