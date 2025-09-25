using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Helpers;

namespace Rosidl.Generator.CSharp.Builders;

public class ActionClassBuilder
{
    private ActionBuildContext _context;

    public ActionClassBuilder(ActionBuildContext context)
    {
        _context = context;
    }

    public CSharpElement Build(string path, bool isInternal, bool generateEvent, bool generateDetails)
    {
        var cls = new CSharpClass(_context.ClassName);

        if (isInternal)
        {
            cls.Visibility = CSharpVisibility.Internal;
        }

        cls.AddTypeSupportAttribute(_context.Metadata);
        cls.AddComments(_context.Metadata);

        cls.Modifiers |= CSharpModifiers.Unsafe;
        cls.BaseTypes.Add(new CSharpFreeType($"global::Rosidl.Runtime.IAction<{_context.Goal.ClassName}, {_context.Result.ClassName}, {_context.Feedback.ClassName}>"));

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

        var ns = new CSharpNamespace(_context.Goal.Namespace);

        ns.Members.Add(cls);
        ns.Members.Add(new MessageClassBuilder(_context.Goal).Build(null, isInternal));
        ns.Members.Add(new MessageClassBuilder(_context.Result).Build(null, isInternal));
        ns.Members.Add(new MessageClassBuilder(_context.Feedback).Build(null, isInternal));

        if (generateDetails)
        {
            // The following types will not be used by application codes.
            // Rcl.NET access these types using message introspection,
            // so there's no need to exposed these types as the Action
            // class will need to take 10 generic arguments to define all of them.
            // 
            // Just generating these for completeness (mainly for service introspection).
            ns.Members.Add(new MessageClassBuilder(_context.FeedbackMessage).Build(null, isInternal));
            new ServiceClassBuilder(_context.SendGoal).Build(ns, isInternal, generateEvent);
            new ServiceClassBuilder(_context.GetResult).Build(ns, isInternal, generateEvent);
        }

        var file = new CSharpGeneratedFile(path);
        file.Members.Clear();
        file.Members.Add(new CSharpFreeMember { Text = "#nullable enable" });
        file.Members.Add(ns);

        return file;
    }

    private CSharpMethod EmitGetTypeSupportHandle()
    {
        var method = new CSharpMethod("GetTypeSupportHandle")
        {
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType("global::Rosidl.Runtime.TypeSupportHandle")
        };

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                return new global::Rosidl.Runtime.TypeSupportHandle(_PInvoke(), global::Rosidl.Runtime.HandleType.Action);

                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{_context.Goal.TypeSupportLibraryName}}", EntryPoint = "rosidl_typesupport_c__get_action_type_support_handle__{{_context.Metadata.Id.Package}}__{{_context.Metadata.Id.SubFolder}}__{{_context.Metadata.Id.Name}}")]
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

public class ActionBuildContext
{
    private static readonly ComplexTypeMetadata UuidType = new ComplexTypeMetadata(new("unique_identifier_msgs", "msg", "UUID"));
    private static readonly ComplexTypeMetadata TimeType = new ComplexTypeMetadata(new("builtin_interfaces", "msg", "Time"));

    public MessageBuildContext Goal { get; }

    public MessageBuildContext Result { get; }

    public MessageBuildContext Feedback { get; }

    public MessageBuildContext FeedbackMessage { get; }

    public ServiceBuildContext SendGoal { get; }

    public ServiceBuildContext GetResult { get; }

    public ActionMetadata Metadata { get; }

    public GeneratorOptions Options { get; }

    public string ClassName { get; }

    public ActionBuildContext(ActionMetadata metadata, GeneratorOptions options, MsgParser parser, object? parentContext = null)
    {
        Metadata = metadata;
        Options = options;
        ClassName = options.ResolveActionClassName(this, metadata);

        Goal = new(
            new(metadata.Id with { Name = metadata.Id.Name + "_Goal" }, metadata.Comments, metadata.GoalFields),
            options, MessageType.ActionGoal, this);
        Goal.Metadata.Version = metadata.Version;

        Result = new(
            new(metadata.Id with { Name = metadata.Id.Name + "_Result" }, metadata.Comments, metadata.ResultFields),
            options, MessageType.ActionResult, this);
        Result.Metadata.Version = metadata.Version;

        Feedback = new(
            new(metadata.Id with { Name = metadata.Id.Name + "_Feedback" }, metadata.Comments, metadata.FeedbackFields),
            options, MessageType.ActionFeedback, this);
        Feedback.Metadata.Version = metadata.Version;

        FeedbackMessage = GenerateFeedbackMessage();
        SendGoal = GenerateSendGoal(parser);
        GetResult = GenerateGetResult(parser);
    }

    public MessageBuildContext GenerateFeedbackMessage()
    {
        return new(
            new(Metadata.Id with { Name = Metadata.Id.Name + "_FeedbackMessage" }, Metadata.Comments,
            new VariableFieldMetadata[]
            {
                new(UuidType, "goal_id", null, null, Array.Empty<string>()),
                new(Feedback.Metadata.Ref(), "feedback", null, null, Array.Empty<string>()),
            }),
            Options, MessageType.ActionFeedbackMessage, this);
    }

    private ServiceBuildContext GenerateSendGoal(MsgParser parser)
    {
        var id = Metadata.Id with { Name = Metadata.Id.Name + "_SendGoal" };
        return new ServiceBuildContext(
            new ServiceMetadata(id, Metadata.Comments,
            new[] {
                new VariableFieldMetadata(UuidType, "goal_id", null, null, Array.Empty<string>()),
                new VariableFieldMetadata(Goal.Metadata.Ref(), "goal", null, null, Array.Empty<string>())
            },
            new[] {
                new VariableFieldMetadata(new PrimitiveTypeMetadata(PrimitiveTypes.Bool, null), "accepted", null, null, Array.Empty<string>()),
                new VariableFieldMetadata(TimeType, "stamp", null, null, Array.Empty<string>())
            },
            parser.EmitServiceEventFields(id)
            ), Options, ServiceType.ActionSendGoal, this);
    }

    private ServiceBuildContext GenerateGetResult(MsgParser parser)
    {
        var id = Metadata.Id with { Name = Metadata.Id.Name + "_GetResult" };
        return new ServiceBuildContext(
            new ServiceMetadata(id, Metadata.Comments,
            new[] {
                new VariableFieldMetadata(UuidType, "goal_id", null,null, Array.Empty<string>())
            },
            new[] {
                new VariableFieldMetadata(new PrimitiveTypeMetadata(PrimitiveTypes.Int8, null), "status", null, null, Array.Empty<string>()),
                new VariableFieldMetadata(Result.Metadata.Ref(), "result", null, null, Array.Empty<string>())
            },
            parser.EmitServiceEventFields(id)
            ), Options, ServiceType.ActionGetResult, this);
    }
}