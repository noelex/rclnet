using System.Collections.ObjectModel;
using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;

namespace Rosidl.Generator.CSharp;

abstract record Statement();

record Error(string message) : Statement;

record Empty() : Statement
{
    public static readonly Empty Default = new();
}

record Seperator() : Statement
{
    public static readonly Seperator Default = new();
}

record Comment(string Content) : Statement;

record InlineComment(string Content) : Comment(Content);

record FieldDeclaration(
    string Type,
    string Identifier,
    string Value,
    bool IsArray,
    int? ArrayLength,
    string? InlineComment,
    bool IsArrayBounded,
    int? ElementBoundSize,
    bool IsConstant) : Statement
{
    public override string ToString()
    {
        var type = Type;
        if (ElementBoundSize != null)
        {
            type += $"<={ElementBoundSize}";
        }
        if (IsArray)
        {
            if (ArrayLength == null)
            {
                type += "[]";
            }
            else
            {
                type += $"[{(IsArrayBounded ? "<=" : "")}{ArrayLength}]";
            }
        }
        return string.IsNullOrWhiteSpace(Value) ? $"{type} {Identifier}" : $"{type} {Identifier}{(IsConstant ? " =" : "")} {Value}";
    }
}

public class MsgParser
{
    private static readonly Regex ConstantFieldPattern
        = new(@"^([A-Za-z]+[A-Za-z0-9_/]*[A-Za-z0-9_]*)\s+([A-Z]+[A-Z0-9_]*)\s*=\s*(.+)$");

    private static readonly Regex VariableFieldPattern
        = new(@"^([A-Za-z]+[A-Za-z0-9_/]*[A-Za-z0-9_]*)(\[(<=)?(\d*)\])?\s+([A-Za-z]+[A-Za-z0-9_]*)(\s+(.+))?$");

    private static readonly Regex StringFieldPattern
        = new(@"^(wstring|string)(<=(\d+))?(\[(<=)?(\d*)\])?\s+([A-Za-z]+[A-Za-z0-9_]*)(\s+(.+))?$");

    private readonly Dictionary<string, PrimitiveTypeMetadata> _primitiveTypeCache = new()
    {
        { "int8", new(PrimitiveTypes.Int8, null) },
        { "int16", new(PrimitiveTypes.Int16, null) },
        { "int32", new(PrimitiveTypes.Int32, null) },
        { "int64", new(PrimitiveTypes.Int64, null) },
        { "uint8", new(PrimitiveTypes.UInt8, null) },
        { "uint16", new(PrimitiveTypes.UInt16, null) },
        { "uint32", new(PrimitiveTypes.UInt32, null) },
        { "uint64", new(PrimitiveTypes.UInt64, null) },
        { "float32", new(PrimitiveTypes.Float32, null) },
        { "float64", new(PrimitiveTypes.Float64, null) },
        { "bool", new(PrimitiveTypes.Bool, null) },
        { "byte", new(PrimitiveTypes.UInt8, null) },
        { "char", new(PrimitiveTypes.Int8, null) },
        { "string", new(PrimitiveTypes.String, null) },
        { "wstring", new(PrimitiveTypes.WString, null) }
    };

    private readonly Dictionary<string, ComplexTypeMetadata> _complexTypeCache = new();

    private static IEnumerable<Statement> Parse(TextReader reader, bool disposeReader = false)
    {
        try
        {
            var currentLine = 0;
            while (true)
            {
                var line = reader.ReadLine();
                if (line is null)
                {
                    break;
                }
                currentLine++;

                if (string.IsNullOrWhiteSpace(line))
                {
                    yield return Empty.Default;
                }
                else if (line == "---")
                {
                    yield return Seperator.Default;
                }
                else
                {
                    line = line.Trim();

                    if (line.StartsWith("#"))
                    {
                        yield return new Comment(line[1..].Trim());
                    }
                    else
                    {
                        var match = ConstantFieldPattern.Match(line);
                        string type, identifier, value, arrayIndex = "", elementIndex = "";
                        bool isArray = false, isArrayBounded = false, isElementBounded = false, isConstant = false;
                        if (match.Success)
                        {
                            type = match.Groups[1].Value;
                            identifier = match.Groups[2].Value;
                            value = match.Groups[3].Value.Trim();
                            isConstant = true;
                        }
                        else
                        {
                            match = VariableFieldPattern.Match(line);
                            if (match.Success)
                            {

                                type = match.Groups[1].Value;
                                identifier = match.Groups[5].Value;
                                value = match.Groups[7].Value.Trim();
                                isArrayBounded = !string.IsNullOrEmpty(match.Groups[3].Value);
                                arrayIndex = match.Groups[4].Value;
                                isArray = !string.IsNullOrEmpty(match.Groups[2].Value);
                            }
                            else
                            {
                                match = StringFieldPattern.Match(line);
                                if (!match.Success)
                                {
                                    yield return new Error($"Unexpected statement at line {currentLine}.");
                                    continue;
                                }

                                type = match.Groups[1].Value;
                                elementIndex = match.Groups[3].Value;
                                isElementBounded = elementIndex.Length > 0;

                                isArray = match.Groups[4].Length > 0;
                                isArrayBounded = match.Groups[5].Length > 0;
                                arrayIndex = match.Groups[6].Value;

                                identifier = match.Groups[7].Value;
                                value = match.Groups[9].Value.Trim();
                            }
                        }

                        var inlineComment = ExtractInlineComment(ref value);
                        yield return new FieldDeclaration(
                            type, identifier, value, isArray,
                            string.IsNullOrEmpty(arrayIndex) ? null : int.Parse(arrayIndex),
                            inlineComment,
                            isArrayBounded,
                            isElementBounded ? int.Parse(elementIndex) : null,
                            isConstant);
                    }
                }
            }
        }
        finally
        {
            if (disposeReader) reader.Dispose();
        }
    }

    private static string? ExtractInlineComment(ref string input)
    {
        var closed = true;
        char currentDelim = '\0';
        var commentStart = -1;

        for (var i = 0; i < input.Length; i++)
        {
            if (closed)
            {
                if (input[i] is '"' or '\'')
                {
                    currentDelim = input[i];
                    closed = false;
                }
                else if (input[i] is '#')
                {
                    commentStart = i;
                    break;
                }
            }
            else if (input[i] == currentDelim && input[i - 1] != '\\')
            {
                closed = true;
            }
        }

        string? inlineComment = null;
        if (commentStart >= 0)
        {
            if (commentStart + 1 < input.Length)
            {
                inlineComment = input[(commentStart + 1)..];
            }

            input = input[..commentStart].Trim();
        }

        return inlineComment?.Trim();
    }

    private static Statement[] ReadStatements(Stream stream, bool leaveOpen, MessageIdentifier id)
    {
        using var reader = new StreamReader(stream, Encoding.UTF8, true, 1024, leaveOpen: leaveOpen);
        return ReadStatements(reader, id);
    }

    private static Statement[] ReadStatements(string input, MessageIdentifier id)
    {
        using var reader = new StringReader(input);
        return ReadStatements(reader, id);
    }

    private static Statement[] ReadStatements(TextReader input, MessageIdentifier id)
    {
        var statements = Parse(input).ToArray();

        var sb = new StringBuilder();
        foreach (var error in statements.OfType<Error>())
        {
            sb.AppendLine();
            sb.Append(" - ");
            sb.Append(error.message);
        }

        if (sb.Length > 0)
        {
            throw new FormatException($"Unable to parse '{id}': {sb}");
        }

        return statements;
    }

    private MessageMetadata ParseMessage(MessageIdentifier id, Statement[] lines)
    {
        if (lines.OfType<Seperator>().Any())
            throw new FormatException("Separator is not allowed in message definition.");

        var fileComments = ExtractFileComments(lines);
        return new(id, fileComments, ExtractFields(id, lines, lines.OfType<FieldDeclaration>()));
    }

    private ServiceMetadata ParseService(MessageIdentifier id, Statement[] lines)
    {
        var seperators = lines.OfType<Seperator>();
        if (seperators.Count() != 1)
        {
            throw new FormatException("Service definition requires exactly one separator.");
        }

        var fileComments = ExtractFileComments(lines);

        var seperator = seperators.First();
        var sepIndex = Array.IndexOf(lines, seperator);

        var requestStatements = lines[0..sepIndex];
        var responseStatements = lines[(sepIndex + 1)..];

        return new(id, fileComments,
            ExtractFields(id, lines, requestStatements),
            ExtractFields(id, lines, responseStatements),
            EmitServiceEventFields(id));
    }

    internal IReadOnlyCollection<FieldMetadata> EmitServiceEventFields(MessageIdentifier id)
    {
        var statements = ReadStatements($"""
            # Event info
            # Contains event type, timestamp, and request ID
            service_msgs/msg/ServiceEventInfo info

            # The actual request content sent or received
            # This field is only set if the event type is REQUEST_SENT or REQUEST_RECEIVED,
            # and the introspection feature is configured to include payload data.
            {id}_Request[<=1] request

            # The actual response content sent or received
            # This field is only set if the event type is RESPONSE_SENT or RESPONSE_RECEIVED,
            # and the introspection feature is configured to include payload data.
            {id}_Response[<=1] response
            """, id);
        var metadata = ParseMessage(id, statements);
        return metadata.Fields;
    }

    private ActionMetadata ParseAction(MessageIdentifier id, Statement[] lines)
    {
        var separators = lines.OfType<Seperator>().ToArray();
        if (separators.Length != 2)
        {
            throw new FormatException("Service definition requires exactly one separator.");
        }

        var fileComments = ExtractFileComments(lines);

        var sepIndex1 = Array.IndexOf(lines, separators[0]);
        var sepIndex2 = Array.LastIndexOf(lines, separators[1]);

        var goalStatements = lines[0..sepIndex1];
        var resultStatements = lines[(sepIndex1 + 1)..sepIndex2];
        var feedbackStatements = lines[(sepIndex2 + 1)..];

        return new(id, fileComments,
            ExtractFields(id, lines, goalStatements),
            ExtractFields(id, lines, feedbackStatements),
            ExtractFields(id, lines, resultStatements));
    }

    public ComplexTypeMetadata Parse(string package, string name, string inputString, string? subFolder = null)
    {
        var statements = ReadStatements(inputString, new(package, subFolder, name));
        var separators = statements.OfType<Seperator>().Count();
        return separators switch
        {
            0 => ParseMessage(new(package, subFolder ?? "msg", name), statements),
            1 => ParseService(new(package, subFolder ?? "srv", name), statements),
            2 => ParseAction(new(package, subFolder ?? "action", name), statements),
            _ => throw new FormatException($"Unexpected count of separators. Expecting 0, 1 or 2, received {separators}.")
        };
    }

    public ComplexTypeMetadata Parse(string package, string name, Stream inputStream, bool leaveOpen = false, string? subFolder = null)
    {
        var statements = ReadStatements(inputStream, leaveOpen, new(package, subFolder, name));
        var separators = statements.OfType<Seperator>().Count();
        return separators switch
        {
            0 => ParseMessage(new(package, subFolder ?? "msg", name), statements),
            1 => ParseService(new(package, subFolder ?? "srv", name), statements),
            2 => ParseAction(new(package, subFolder ?? "action", name), statements),
            _ => throw new FormatException($"Unexpected count of separators. Expecting 0, 1 or 2, received {separators}.")
        };
    }

    private IReadOnlyCollection<FieldMetadata> ExtractFields(MessageIdentifier id, Statement[] lines, IEnumerable<Statement> statements)
    {
        var list = new List<FieldMetadata>();
        foreach (var field in statements.OfType<FieldDeclaration>())
        {
            var comments = ExtractFieldComments(lines, field);
            try
            {
                list.Add(CreateField(id, field, comments));
            }
            catch (Exception e)
            {
                throw new FormatException($"Unable to parse field defined as '{field}' in '{id}': " + e.Message, e);
            }
        }

        return new ReadOnlyCollection<FieldMetadata>(list);
    }

    private FieldMetadata CreateField(MessageIdentifier id, FieldDeclaration declaration, string[] comments)
    {
        var type = CreateFieldType(id, declaration);
        if (IsConstantField(declaration.Identifier))
        {
            if (type is PrimitiveTypeMetadata p)
            {
                return CreateConstantField(p, declaration, comments);
            }

            throw new FormatException($"Invalid constant value type '{type}'.");
        }
        else if (IsVariablField(declaration.Identifier))
        {
            return CreateVariableField(type, declaration, comments);
        }

        throw new FormatException($"Invalid field {(declaration.IsConstant ? "constant" : "variable")} identifier '{declaration.Identifier}'.");
    }

    private TypeMetadata CreateFieldType(MessageIdentifier id, FieldDeclaration declaration)
    {
        TypeMetadata metadata;

        // Special case for string with upper boundary
        if (declaration.Type == "string" && declaration.ElementBoundSize != null)
        {
            var elementType = new PrimitiveTypeMetadata(PrimitiveTypes.String, declaration.ElementBoundSize);
            return declaration.IsArray ? new ArrayTypeMetadata(elementType, declaration.ArrayLength, declaration.IsArrayBounded) : elementType;
        }
        else if (declaration.Type == "wstring" && declaration.ElementBoundSize != null)
        {
            var elementType = new PrimitiveTypeMetadata(PrimitiveTypes.WString, declaration.ElementBoundSize);
            return declaration.IsArray ? new ArrayTypeMetadata(elementType, declaration.ArrayLength, declaration.IsArrayBounded) : elementType;
        }

        if (!_primitiveTypeCache.TryGetValue(declaration.Type, out var t))
        {
            var fullName = declaration.Type.Contains('/')
                ? declaration.Type : $"{id.Package}/msg/{declaration.Type}";
            if (!_complexTypeCache.TryGetValue(fullName, out var type))
            {
                var parts = fullName.Split('/');
                var subdir = parts.Length == 3 ? parts[1] : "msg";
                type = new ComplexTypeMetadata(new MessageIdentifier(parts[0], subdir, parts[^1]));
                _complexTypeCache[fullName] = type;
            }

            metadata = type;
        }
        else
        {
            metadata = t;
        }

        return declaration.IsArray ? new ArrayTypeMetadata(metadata, declaration.ArrayLength, declaration.IsArrayBounded) : metadata;
    }

    private ConstantFieldMetadata CreateConstantField(PrimitiveTypeMetadata typeHint, FieldDeclaration declaration, string[] comments)
    {
        object val = CreatePrimitiveValue(typeHint, declaration.Value);
        return new ConstantFieldMetadata(typeHint, declaration.Identifier, val, declaration.Value, comments);
    }

    private VariableFieldMetadata CreateVariableField(TypeMetadata typeHint, FieldDeclaration declaration, string[] comments)
    {
        if (string.IsNullOrEmpty(declaration.Value))
        {
            return new VariableFieldMetadata(typeHint, declaration.Identifier, null, null, comments);
        }

        return new VariableFieldMetadata(typeHint, declaration.Identifier, ParseValue(typeHint, declaration.Value), declaration.Value, comments);
    }

    private object CreatePrimitiveValue(PrimitiveTypeMetadata typeHint, string value)
    {
        return typeHint.ValueType switch
        {
            PrimitiveTypes.Int8 => value.StartsWith("0x") ? sbyte.Parse(value[2..], NumberStyles.HexNumber) : sbyte.Parse(value),
            PrimitiveTypes.Int16 => value.StartsWith("0x") ? short.Parse(value[2..], NumberStyles.HexNumber) : short.Parse(value),
            PrimitiveTypes.Int32 => value.StartsWith("0x") ? int.Parse(value[2..], NumberStyles.HexNumber) : int.Parse(value),
            PrimitiveTypes.Int64 => value.StartsWith("0x") ? long.Parse(value[2..], NumberStyles.HexNumber) : long.Parse(value),
            PrimitiveTypes.UInt8 => value.StartsWith("0x") ? byte.Parse(value[2..], NumberStyles.HexNumber) : byte.Parse(value),
            PrimitiveTypes.UInt16 => value.StartsWith("0x") ? ushort.Parse(value[2..], NumberStyles.HexNumber) : ushort.Parse(value),
            PrimitiveTypes.UInt32 => value.StartsWith("0x") ? uint.Parse(value[2..], NumberStyles.HexNumber) : uint.Parse(value),
            PrimitiveTypes.UInt64 => value.StartsWith("0x") ? ulong.Parse(value[2..], NumberStyles.HexNumber) : ulong.Parse(value),
            PrimitiveTypes.Float32 => float.Parse(value),
            PrimitiveTypes.Float64 => double.Parse(value),
            PrimitiveTypes.String or PrimitiveTypes.WString => ParseString(value, typeHint.MaxLength),
            PrimitiveTypes.Bool when value is "0" or "false" => false,
            PrimitiveTypes.Bool when value is "1" or "true" => true,
            PrimitiveTypes.Bool => throw new FormatException($"Unexpected boolean value '{value}'."),
            _ => throw new FormatException($"Invalid primitive type '{typeHint.ValueType}'.")
        };
    }

    private object ParseValue(TypeMetadata typeHint, string value)
    {
        if (typeHint is PrimitiveTypeMetadata p)
        {
            return CreatePrimitiveValue(p, value);
        }
        else if (typeHint is ArrayTypeMetadata a)
        {
            return ParseArray(a, value);
        }

        throw new FormatException($"Setting default value for complex type is not supported.");
    }

    private object[] ParseArray(ArrayTypeMetadata typeHint, string value)
    {
        if (value[0] != '[' || value[^1] != ']')
        {
            throw new FormatException($"Invalid array literal: {value}");
        }

        value = value[1..^1].Trim();
        if (typeHint.ElementType is PrimitiveTypeMetadata p)
        {
            var ret = p.ValueType is not PrimitiveTypes.String and not PrimitiveTypes.WString ?
                value.Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries)
                    .Select(x => ParseValue(p, x.Trim())).ToArray()
                : ExtractStringTokens(value).Select(x => ParseString(x, p.MaxLength)).ToArray();

            // Check size of the array
            if (typeHint.Length != null)
            {
                if (!typeHint.IsUpperBounded)
                {
                    if (typeHint.Length != ret.Length)
                    {
                        throw new FormatException($"Default value of type '{typeHint}' requires {typeHint.Length} elements, but {ret.Length} found.");
                    }
                }
                else
                {
                    if (ret.Length > typeHint.Length)
                    {
                        throw new FormatException($"Length of the default value of type '{typeHint}' exceeds the limit of {typeHint.Length}.");
                    }
                }
            }

            return ret;
        }

        throw new FormatException($"Setting default value for complex type is not supported.");
    }

    private static IEnumerable<string> ExtractStringTokens(string input)
    {
        var started = false;
        var start = -1;
        char delim = default;

        var commaEncountered = true;
        for (int i = 0; i < input.Length; i++)
        {
            if (!started)
            {
                if (!commaEncountered)
                {
                    if (input[i] == ',')
                    {
                        commaEncountered = true;
                    }
                    else if (!char.IsWhiteSpace(input[i]))
                    {
                        throw new FormatException($"Unexpected character '{input[i]}' at position {i} in array: [{input}]");
                    }

                    continue;
                }

                if (char.IsWhiteSpace(input[i]))
                {
                    continue;
                }

                delim = input[i];
                start = i;
                if (delim is not '\'' and not '"')
                    throw new FormatException("Unexpected string delimiter.");
                started = true;
            }
            else
            {
                if (input[i] == delim && input[i - 1] != '\\')
                {
                    started = false;
                    commaEncountered = false;
                    yield return input[start..(i + 1)];
                }
            }
        }

        if (started)
        {
            throw new FormatException($"Incomplete string literal found in array: [{input}]");
        }
    }

    private static string ParseString(string input, int? maxLength)
    {
        char delim;
        if (input[0] is '\'' && input[^1] is '\'')
        {
            delim = '\'';
        }
        else if (input[0] is '"' && input[^1] is '"')
        {
            delim = '"';
        }
        else
        {
            throw new FormatException($"Invalid string literal with unexpected delimiter: {input}");
        }

        var result = input[1..^1]
            .Replace(@"\t", "\t")
            .Replace(@"\n", "\n")
            .Replace(@"\r", "\r")
            .Replace(@"\f", "\f")
            .Replace(@"\b", "\b")
            .Replace(@"\\", "\\")
            .Replace($@"\{delim}", delim.ToString());

        if (maxLength != null && result.Length > maxLength)
        {
            throw new FormatException($"Length of the string '{result}' exceeds the limit of {maxLength}.");
        }

        return result;
    }

    private static bool IsConstantField(string identifier)
        => !identifier.StartsWith("_")
           && !char.IsDigit(identifier[0])
           && identifier.All(x => char.IsUpper(x) || char.IsDigit(x) || x is '_')
           && !identifier.Contains("__");

    private static bool IsVariablField(string identifier)
        => !identifier.StartsWith("_")
           && !char.IsDigit(identifier[0])
           && identifier.All(x => char.IsLower(x) || char.IsDigit(x) || x is '_')
           && !identifier.Contains("__");

    private static string[] ExtractFileComments(Statement[] statements)
    {
        var result = new List<string>();
        foreach (var s in statements)
        {
            if (s is Comment c)
            {
                result.Add(c.Content);
            }
            else
            {
                break;
            }
        }

        return result.ToArray();
    }

    private static string[] ExtractFieldComments(Statement[] statements, FieldDeclaration field)
    {
        if (!string.IsNullOrEmpty(field.InlineComment))
        {
            return new[] { field.InlineComment! };
        }

        var result = new List<string>();
        var index = Array.IndexOf(statements, field);

        for (int i = index - 1; i >= 0; i--)
        {
            if (statements[i] is Comment c)
            {
                result.Add(c.Content);
            }
            else
            {
                break;
            }
        }

        result.Reverse();
        return result.ToArray();
    }
}