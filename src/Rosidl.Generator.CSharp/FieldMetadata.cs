namespace Rosidl.Generator.CSharp;

public abstract record FieldMetadata(
    TypeMetadata Type,
    string Name,
    string[] Comments
);

public record VariableFieldMetadata(TypeMetadata Type, string Name, object? DefaultValue, string[] Comments) : FieldMetadata(Type, Name, Comments)
{
    public override string ToString()
        => DefaultValue is null ? $"{Type} {Name}" : $"{Type} {Name} {ToString(DefaultValue)}";

    private static string ToString(object value)
    {
        if (value is string[] s)
        {
            return $"[{string.Join(", ", s.Select(x => string.Format("\"{0}\"", x.Escape())))}]";
        }
        else if (value is object[] o)
        {
            return $"[{string.Join(", ", o)}]";
        }
        else
        {
            return value.ToString() ?? string.Empty;
        }
    }
}

public record ConstantFieldMetadata(TypeMetadata Type, string Name, object Value, string[] Comments) : FieldMetadata(Type, Name, Comments)
{
    public override string ToString()
        => $"{Type} {Name} = {Value}";
}