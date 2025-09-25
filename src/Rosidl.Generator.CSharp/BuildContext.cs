using CppAst.CodeGen.CSharp;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;

namespace Rosidl.Generator.CSharp;

static class Attributes
{
    public static readonly CSharpAttribute SuppressGCTransition
        = new CSharpFreeAttribute("global::System.Runtime.InteropServices.SuppressGCTransitionAttribute");

    public static readonly CSharpAttribute StructLayoutSequential
        = new CSharpFreeAttribute("global::System.Runtime.InteropServices.StructLayoutAttribute(global::System.Runtime.InteropServices.LayoutKind.Sequential)");

    public static readonly CSharpAttribute CallerMemberName
        = new CSharpFreeAttribute($"global::{typeof(CallerMemberNameAttribute).FullName}");

    public static readonly CSharpAttribute DebuggerNonUserCode
        = new CSharpFreeAttribute("global::System.Diagnostics.DebuggerNonUserCodeAttribute");

    public static readonly CSharpAttribute GeneratedCode
        = new CSharpFreeAttribute($"global::System.CodeDom.Compiler.GeneratedCodeAttribute(\"ros2cs\", " +
            $"\"{typeof(Attributes).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()!
                .InformationalVersion}\")");
}

public record VariableFieldInfo(
    VariableFieldMetadata Metadata,
    string ClassName,
    string PrivStructName,
    string PrivStructSequenceName,
        string PropertyName,
        string ParameterName, bool IsNullable, string DefaultValueLiteral);

public enum FieldSite
{
    Class,
    PrivStruct
}

public enum NameType
{
    Class,
    PrivStruct,
    PrivStructSequence
}

public enum MessageType
{
    Plain,
    ServiceRequest,
    ServiceResponse,
    ServiceEvent,
    ActionGoal,
    ActionFeedback,
    ActionResult,
    ActionFeedbackMessage
}

public class MessageBuildContext
{
    private static readonly Dictionary<PrimitiveTypes, string> _primitiveTypeMap = new()
    {
        {PrimitiveTypes.Bool, "bool" },
        {PrimitiveTypes.Int8, "sbyte" },
        {PrimitiveTypes.Int16, "short" },
        {PrimitiveTypes.Int32, "int" },
        {PrimitiveTypes.Int64, "long" },
        {PrimitiveTypes.UInt8, "byte" },
        {PrimitiveTypes.UInt16, "ushort" },
        {PrimitiveTypes.UInt32, "uint" },
        {PrimitiveTypes.UInt64, "ulong" },
        {PrimitiveTypes.Float32, "float" },
        {PrimitiveTypes.Float64, "double" },
        {PrimitiveTypes.String, "string" },
        {PrimitiveTypes.WString, "string" },
    };

    public GeneratorOptions Options { get; set; }

    public MessageMetadata Metadata { get; }

    public string PrivStructName { get; }

    public string PrivStructNameFullyQualified { get; }

    public string PrivStructSequenceName { get; }

    public string PrivStructSequenceNameFullyQualified { get; }

    public string ClassName { get; }

    public string ClassNameFullyQualified { get; }

    public string GeneratorLibraryName { get; }

    public string TypeSupportLibraryName { get; }

    public VariableFieldMetadata[] Variables { get; }

    public ConstantFieldMetadata[] Constants { get; }

    public string Namespace => GetNamespace(Metadata);

    public MessageType Type { get; }

    public object? ParentContext { get; }

    public MessageBuildContext(MessageMetadata metadata, GeneratorOptions options, MessageType type = MessageType.Plain, object? parent = null)
    {
        Type = type;
        ParentContext = parent;

        Metadata = metadata;
        Options = options;
        Variables = metadata.Fields.OfType<VariableFieldMetadata>().ToArray();
        Constants = metadata.Fields.OfType<ConstantFieldMetadata>().ToArray();

        ClassName = Options.ResolveMessageClassName(this, Metadata);
        ClassNameFullyQualified = GetMessageClassReferenceName(Metadata);
        PrivStructName = Options.ResolveMessagePrivStructName(this, Metadata);
        PrivStructNameFullyQualified = GetMessagePrivStructReferenceName(Metadata);
        PrivStructSequenceName = Options.ResolveMessagePrivStructSequenceName(this, Metadata);
        PrivStructSequenceNameFullyQualified = GetMessagePrivStructSequenceReferenceName(Metadata);

        GeneratorLibraryName = Metadata.Id.Package + "__rosidl_generator_c";
        TypeSupportLibraryName = Metadata.Id.Package + "__rosidl_typesupport_c";
    }

    public string GetNativeMessageFunctionSymbol(string function)
    {
        return $"{Metadata.Id.Package}__{Metadata.Id.SubFolder}__{Metadata.Id.Name}__{function}";
    }

    public string GetNativeSequenceFunctionSymbol(string function)
    {
        return $"{Metadata.Id.Package}__{Metadata.Id.SubFolder}__{Metadata.Id.Name}__Sequence__{function}";
    }


    /// <summary>
    /// Returns the namespace of specified type, e.g. <c>MyGeneratedMessages.Geometry.Messages</c>
    /// </summary>
    /// <param name="metadata"></param>
    /// <returns></returns>
    private string GetNamespace(ComplexTypeMetadata metadata)
    {
        var ns = Options.ResolveNamespace(metadata.Id.Package);
        return $"{ns}.{Options.ResolvePackageName(metadata.Id.Package)}";
    }

    /// <summary>
    /// Returns the name of the specified message including its namespace, e.g. <c>MyGeneratedMessages.Geometry.Messages.Vector3</c>
    /// </summary>
    /// <param name="metadata"></param>
    /// <returns></returns>
    private string GetMessageReferenceName(ComplexTypeMetadata metadata, NameType type)
    {
        var cls = Options.ResolveMessageClassName(this, metadata);
        string className = type switch
        {
            NameType.Class => cls,
            NameType.PrivStruct => $"{cls}.{Options.ResolveMessagePrivStructName(this, metadata)}",
            NameType.PrivStructSequence => $"{cls}.{Options.ResolveMessagePrivStructSequenceName(this, metadata)}",
            _ => throw new NotImplementedException()
        };

        var ns = metadata.Id.Package is null
            ? GetNamespace(Metadata) : GetNamespace(metadata);

        return $"global::{ns}.{className}";
    }

    public string GetMessageClassReferenceName(ComplexTypeMetadata metadata) => GetMessageReferenceName(metadata, NameType.Class);

    public string GetMessagePrivStructReferenceName(ComplexTypeMetadata metadata) => GetMessageReferenceName(metadata, NameType.PrivStruct);

    public string GetMessagePrivStructSequenceReferenceName(ComplexTypeMetadata metadata) => GetMessageReferenceName(metadata, NameType.PrivStructSequence);

    public string GetNormalizedFieldName(string name)
    {
        if (name == ClassName || name == PrivStructName || name == PrivStructSequenceName)
        {
            return name + "_";
        }
        return name;
    }

    public string GetNormalizedFieldName(FieldMetadata metadata)
    {
        var name = Options.ResolveFieldName(this, metadata);
        return GetNormalizedFieldName(name);
    }

    public string GetPrimitiveTypeName(PrimitiveTypeMetadata metadata)
    {
        return _primitiveTypeMap[metadata.ValueType];
    }
}

public abstract class MethodBuildContext
{
    public MethodBuildContext(MessageBuildContext messageContext,
        CSharpFreeType structType,
        string structFullyQualifiedName)
    {
        MessageContext = messageContext;
        StructType = structType;
        StructFullyQualifiedName = structFullyQualifiedName;
    }

    public MessageBuildContext MessageContext { get; }

    public CSharpFreeType StructType { get; }

    public string StructName => StructType.ToFullString();

    public string StructFullyQualifiedName { get; }
}

public class PrivStructMethodBuildContext : MethodBuildContext
{
    public PrivStructMethodBuildContext(MessageBuildContext messageContext)
        : base(messageContext,
            new CSharpFreeType(messageContext.PrivStructName),
            messageContext.PrivStructNameFullyQualified)
    {
    }
}

public class SequenceStructMethodBuildContext : MethodBuildContext
{
    public SequenceStructMethodBuildContext(MessageBuildContext messageContext)
        : base(messageContext,
            new CSharpFreeType(messageContext.PrivStructSequenceName),
            messageContext.PrivStructSequenceNameFullyQualified)
    {
    }
}
