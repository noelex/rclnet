using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Helpers;

namespace Rosidl.Generator.CSharp.Builders;

public class ServiceClassBuilder
{
    private ServiceBuildContext _context;

    public ServiceClassBuilder(ServiceBuildContext context)
    {
        _context = context;
    }

    private CSharpElement[] Build(bool isInternal, bool generateEvent)
    {
        var cls = new CSharpClass(_context.ClassName);
        if (isInternal)
        {
            cls.Visibility = CSharpVisibility.Internal;
        }

        cls.AddComments(_context.Metadata);
        cls.AddTypeSupportAttribute(_context.Metadata.ToString());
        cls.Modifiers |= CSharpModifiers.Unsafe;
        cls.BaseTypes.Add(new CSharpFreeType($"global::Rosidl.Runtime.IService<{_context.Request.ClassName}, {_context.Response.ClassName}>"));

        foreach (var m in cls.Members.OfType<CSharpMethod>())
        {
            m.Attributes.Add(Attributes.DebuggerNonUserCode);
            m.Attributes.Add(Attributes.GeneratedCode);
        }

        cls.Members.Add(EmitTypeSupportNameProperty());
        cls.Members.Add(EmitGetTypeSupportHandle());

        foreach (var m in cls.Members.OfType<CSharpMethod>())
        {
            m.Attributes.Add(Attributes.DebuggerNonUserCode);
            m.Attributes.Add(Attributes.GeneratedCode);
        }

        var items= new CSharpElement[]
        {
            cls,
            new MessageClassBuilder(_context.Request).Build(null, isInternal),
            new MessageClassBuilder(_context.Response).Build(null, isInternal),
        };

        if (generateEvent)
        {
            return items.Append(new MessageClassBuilder(_context.Event).Build(null, isInternal)).ToArray();
        }
        return items;
    }
    public CSharpElement Build(string path, bool isInternal, bool generateEvent)
    {
        var ns = new CSharpNamespace(_context.Request.Namespace);

        foreach (var item in Build(isInternal, generateEvent))
        {
            ns.Members.Add(item);
        }

        var file = new CSharpGeneratedFile(path);
        file.Members.Clear();
        file.Members.Add(new CSharpFreeMember { Text = "#nullable enable" });
        file.Members.Add(ns);

        return file;
    }

    public void Build(CSharpNamespace ns, bool isInternal, bool generateEvent)
    {
        foreach (var item in Build(isInternal, generateEvent))
        {
            ns.Members.Add(item);
        }
    }

    private CSharpMethod EmitGetTypeSupportHandle()
    {
        var method = new CSharpMethod()
        {
            Name = "GetTypeSupportHandle",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType("global::Rosidl.Runtime.TypeSupportHandle")
        };

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                return new global::Rosidl.Runtime.TypeSupportHandle(_PInvoke(), global::Rosidl.Runtime.HandleType.Service);

                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{_context.Request.TypeSupportLibraryName}}", EntryPoint = "rosidl_typesupport_c__get_service_type_support_handle__{{_context.Metadata.Package}}__{{_context.Metadata.SubFolder}}__{{_context.Metadata.Name}}")]
                static extern nint _PInvoke();
                """);
        };
        return method;
    }

    private CSharpProperty EmitTypeSupportNameProperty()
    {
        var prop = new CSharpProperty("TypeSupportName")
        {
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.String)
        };

        prop.Attributes.Add(Attributes.DebuggerNonUserCode);
        prop.Attributes.Add(Attributes.GeneratedCode);

        prop.GetBodyInlined = $"\"{_context.Metadata}\"";
        return prop;
    }
}


public enum ServiceType
{
    Plain,
    ActionSendGoal,
    ActionGetResult,
}

public class ServiceBuildContext
{
    public MessageBuildContext Request { get; }

    public MessageBuildContext Response { get; }

    public MessageBuildContext Event { get; }

    public ServiceMetadata Metadata { get; }

    public GeneratorOptions Options { get; }

    public string ClassName { get; }

    public ServiceType Type { get; }

    public object? ParentContext { get; }

    public ServiceBuildContext(ServiceMetadata metadata, GeneratorOptions options, ServiceType type = ServiceType.Plain, object? parentContext = null)
    {
        Type = type;
        ParentContext = parentContext;
        Metadata = metadata;
        Options = options;
        ClassName = options.ResolveServiceClassName(this, metadata);

        Request = new(
            new(metadata.Package, metadata.SubFolder, metadata.Name + "_Request", metadata.Comments, metadata.RequestFields),
            options, MessageType.ServiceRequest, this);

        Response = new(
            new(metadata.Package, metadata.SubFolder, metadata.Name + "_Response", metadata.Comments, metadata.ResponseFields),
            options, MessageType.ServiceResponse, this);

        Event = new(
            new(metadata.Package, metadata.SubFolder, metadata.Name + "_Event", metadata.Comments, metadata.EventFields),
            options, MessageType.ServiceEvent, this);
    }
}