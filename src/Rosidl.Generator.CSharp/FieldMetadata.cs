namespace Rosidl.Generator.CSharp;

public abstract record FieldMetadata(
    TypeMetadata Type,
    string Name,
    string[] Comments
)
{
    protected static string ToString(object value)
    {
        if (value is string sv)
        {
            return string.Format("\"{0}\"", sv.Escape());
        }
        else if (value is bool bv)
        {
            return bv ? "true" : "false";
        }
        else if (value is object[] o)
        {
            return $"[{string.Join(", ", o.Select(ToString))}]";
        }
        else
        {
            return value.ToString() ?? string.Empty;
        }
    }
}

public record VariableFieldMetadata(TypeMetadata Type, string Name, object? DefaultValue, string? DefaultValueLiteral, string[] Comments) : FieldMetadata(Type, Name, Comments)
{
    public override string ToString()
        => DefaultValueLiteral is null ? $"{Type} {Name}" : $"{Type} {Name} {DefaultValueLiteral}";

}

public record ConstantFieldMetadata(TypeMetadata Type, string Name, object Value, string ValueLiteral, string[] Comments) : FieldMetadata(Type, Name, Comments)
{
    public override string ToString()
        => $"{Type} {Name} = {ValueLiteral}";
}