using CppAst.CodeGen.CSharp;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;

namespace Rosidl.Generator.CSharp;

static class Attributes
{
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

    public static CSharpAttribute RosidlAbi(NativeLayout layout) =>
        new CSharpFreeAttribute(
            $"global::Rosidl.Runtime.RosidlAbiAttribute(global::Rosidl.Runtime.RosidlNativeAbi.{layout})");
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

internal enum NativeLayout
{
    V1,
    V2
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
    private static readonly Dictionary<PrimitiveTypes, string> s_primitiveTypeMap = new()
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

    internal NativeLayoutBuildContext NativeLayout => NativeLayouts[0];

    internal IReadOnlyList<NativeLayoutBuildContext> NativeLayouts { get; }

    public string PrivStructName => NativeLayout.PrivName;

    public string PrivStructNameFullyQualified => NativeLayout.PrivNameFullyQualified;

    public string PrivStructSequenceName => NativeLayout.PrivSequenceName;

    public string PrivStructSequenceNameFullyQualified => NativeLayout.PrivSequenceNameFullyQualified;

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
        NativeLayouts = Options.Abi switch
        {
            RosidlAbiMode.V1 => [CreateNativeLayout(global::Rosidl.Generator.CSharp.NativeLayout.V1)],
            RosidlAbiMode.V2 => [CreateNativeLayout(global::Rosidl.Generator.CSharp.NativeLayout.V2)],
            RosidlAbiMode.Portable =>
            [
                CreateNativeLayout(global::Rosidl.Generator.CSharp.NativeLayout.V1),
                CreateNativeLayout(global::Rosidl.Generator.CSharp.NativeLayout.V2, "V2")
            ],
            _ => throw new NotSupportedException(),
        };

        GeneratorLibraryName = Metadata.Id.Package + "__rosidl_generator_c";
        TypeSupportLibraryName = Metadata.Id.Package + "__rosidl_typesupport_c";

        NativeLayoutBuildContext CreateNativeLayout(NativeLayout layout, string suffix = "")
        {
            return new NativeLayoutBuildContext(
                this,
                layout,
                (context, metadata) => Options.ResolveMessagePrivStructName(context, metadata) + suffix,
                (context, metadata) => Options.ResolveMessagePrivStructSequenceName(context, metadata) + suffix);
        }
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
    private string GetMessageReferenceName(ComplexTypeMetadata metadata, string? nestedTypeName = null)
    {
        var cls = Options.ResolveMessageClassName(this, metadata);
        var className = nestedTypeName is null ? cls : $"{cls}.{nestedTypeName}";

        var ns = metadata.Id.Package is null
            ? GetNamespace(Metadata) : GetNamespace(metadata);

        return $"global::{ns}.{className}";
    }

    public string GetMessageClassReferenceName(ComplexTypeMetadata metadata) => GetMessageReferenceName(metadata);

    public string GetMessagePrivStructReferenceName(ComplexTypeMetadata metadata) =>
        NativeLayout.GetMessagePrivStructReferenceName(metadata);

    public string GetMessagePrivStructSequenceReferenceName(ComplexTypeMetadata metadata) =>
        NativeLayout.GetMessagePrivStructSequenceReferenceName(metadata);

    internal string GetMessageNestedTypeReferenceName(ComplexTypeMetadata metadata, string nestedTypeName) =>
        GetMessageReferenceName(metadata, nestedTypeName);

    public string GetNormalizedFieldName(string name)
    {
        if (name == ClassName || NativeLayouts.Any(x => name == x.PrivName || name == x.PrivSequenceName))
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
        return s_primitiveTypeMap[metadata.ValueType];
    }
}

internal sealed class NativeLayoutBuildContext
{
    private readonly Func<MessageBuildContext, ComplexTypeMetadata, string> _resolvePrivName;
    private readonly Func<MessageBuildContext, ComplexTypeMetadata, string> _resolvePrivSequenceName;

    public NativeLayoutBuildContext(
        MessageBuildContext messageContext,
        NativeLayout layout,
        Func<MessageBuildContext, ComplexTypeMetadata, string> resolvePrivName,
        Func<MessageBuildContext, ComplexTypeMetadata, string> resolvePrivSequenceName)
    {
        MessageContext = messageContext;
        Layout = layout;
        _resolvePrivName = resolvePrivName;
        _resolvePrivSequenceName = resolvePrivSequenceName;
        PrivName = resolvePrivName(messageContext, messageContext.Metadata);
        PrivSequenceName = resolvePrivSequenceName(messageContext, messageContext.Metadata);
    }

    public MessageBuildContext MessageContext { get; }

    public NativeLayout Layout { get; }

    public string PrivName { get; }

    public string PrivNameFullyQualified => GetMessagePrivStructReferenceName(MessageContext.Metadata);

    public string PrivSequenceName { get; }

    public string PrivSequenceNameFullyQualified => GetMessagePrivStructSequenceReferenceName(MessageContext.Metadata);

    public string NativeAbiExpression => $"global::Rosidl.Runtime.RosidlNativeAbi.{Layout}";

    public string RequireNativeAbiStatement =>
        $"global::Rosidl.Runtime.RosidlRuntime.RequireNativeAbi({NativeAbiExpression});";

    public string GetPrimitiveSequenceTypeName(PrimitiveTypeMetadata metadata)
    {
        var name = metadata.ValueType switch
        {
            PrimitiveTypes.Bool => "BooleanSequence",
            PrimitiveTypes.Float32 => "FloatSequence",
            PrimitiveTypes.Float64 => "DoubleSequence",
            PrimitiveTypes.Int16 => "Int16Sequence",
            PrimitiveTypes.Int32 => "Int32Sequence",
            PrimitiveTypes.Int64 => "Int64Sequence",
            PrimitiveTypes.Int8 => "Int8Sequence",
            PrimitiveTypes.UInt16 => "UInt16Sequence",
            PrimitiveTypes.UInt32 => "UInt32Sequence",
            PrimitiveTypes.UInt64 => "UInt64Sequence",
            PrimitiveTypes.UInt8 => "UInt8Sequence",
            PrimitiveTypes.String => "CStringSequence",
            PrimitiveTypes.WString => "U16StringSequence",
            _ => throw new NotSupportedException(),
        };

        var suffix = Layout == NativeLayout.V2 ? "V2" : string.Empty;
        return $"global::Rosidl.Runtime.Interop.{name}{suffix}";
    }

    public string GetMessagePrivStructReferenceName(ComplexTypeMetadata metadata) =>
        MessageContext.GetMessageNestedTypeReferenceName(metadata, _resolvePrivName(MessageContext, metadata));

    public string GetMessagePrivStructSequenceReferenceName(ComplexTypeMetadata metadata) =>
        MessageContext.GetMessageNestedTypeReferenceName(metadata, _resolvePrivSequenceName(MessageContext, metadata));
}

public abstract class MethodBuildContext
{
    public MethodBuildContext(MessageBuildContext messageContext,
        CSharpFreeType structType,
        string structFullyQualifiedName)
        : this(messageContext.NativeLayout, structType, structFullyQualifiedName)
    {
    }

    internal MethodBuildContext(NativeLayoutBuildContext nativeLayoutContext,
        CSharpFreeType structType,
        string structFullyQualifiedName)
    {
        NativeLayoutContext = nativeLayoutContext;
        StructType = structType;
        StructFullyQualifiedName = structFullyQualifiedName;
    }

    internal NativeLayoutBuildContext NativeLayoutContext { get; }

    public MessageBuildContext MessageContext => NativeLayoutContext.MessageContext;

    public CSharpFreeType StructType { get; }

    public string StructName => StructType.ToFullString();

    public string StructFullyQualifiedName { get; }

    internal bool IsSequence => this is SequenceStructMethodBuildContext;

}

public class PrivStructMethodBuildContext : MethodBuildContext
{
    public PrivStructMethodBuildContext(MessageBuildContext messageContext)
        : this(messageContext.NativeLayout)
    {
    }

    internal PrivStructMethodBuildContext(NativeLayoutBuildContext nativeLayoutContext)
        : base(nativeLayoutContext,
            new CSharpFreeType(nativeLayoutContext.PrivName),
            nativeLayoutContext.PrivNameFullyQualified)
    {
    }
}

public class SequenceStructMethodBuildContext : MethodBuildContext
{
    public SequenceStructMethodBuildContext(MessageBuildContext messageContext)
        : this(messageContext.NativeLayout)
    {
    }

    internal SequenceStructMethodBuildContext(NativeLayoutBuildContext nativeLayoutContext)
        : base(nativeLayoutContext,
            new CSharpFreeType(nativeLayoutContext.PrivSequenceName),
            nativeLayoutContext.PrivSequenceNameFullyQualified)
    {
    }
}
