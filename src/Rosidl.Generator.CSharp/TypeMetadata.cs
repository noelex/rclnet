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

public record ComplexTypeMetadata(string Package, string SubFolder, string Name) : TypeMetadata
{
    public override string ToString()
    {
        return string.IsNullOrEmpty(SubFolder) ?
            $"{Package}/{Name}" : $"{Package}/{SubFolder}/{Name}";
    }
}

public record MessageMetadata(
    string Package,
    string SubFolder,
    string Name,
    string[] Comments,
    IReadOnlyCollection<FieldMetadata> Fields
) : ComplexTypeMetadata(Package, SubFolder, Name)
{
    public override string ToString()
    {
        return string.IsNullOrEmpty(SubFolder) ?
            $"{Package}/{Name}" : $"{Package}/{SubFolder}/{Name}";
    }
}

public record ServiceMetadata(
    string Package,
    string SubFolder,
    string Name,
    string[] Comments,
    IReadOnlyCollection<FieldMetadata> RequestFields,
    IReadOnlyCollection<FieldMetadata> ResponseFields
) : ComplexTypeMetadata(Package, SubFolder, Name)
{
    public override string ToString()
    {
        return string.IsNullOrEmpty(SubFolder) ?
            $"{Package}/{Name}" : $"{Package}/{SubFolder}/{Name}";
    }
}

public record ActionMetadata(
    string Package,
    string SubFolder,
    string Name,
    string[] Comments,
    IReadOnlyCollection<FieldMetadata> GoalFields,
    IReadOnlyCollection<FieldMetadata> FeedbackFields,
    IReadOnlyCollection<FieldMetadata> ResultFields
) : ComplexTypeMetadata(Package, SubFolder, Name)
{
    public override string ToString()
    {
        return string.IsNullOrEmpty(SubFolder) ?
            $"{Package}/{Name}" : $"{Package}/{SubFolder}/{Name}";
    }
}