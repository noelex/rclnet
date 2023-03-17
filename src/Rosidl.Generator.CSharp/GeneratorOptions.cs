using Rosidl.Generator.CSharp.Builders;

namespace Rosidl.Generator.CSharp;

public class GeneratorOptions
{
    public GeneratorOptions()
    {
        ResolveNamespace = _ => RootNamespace;
    }

    public string RootNamespace { get; set; } = "Rosidl.Messages";

    public Func<string, string> ResolveNamespace { get; set; }

    public Func<string, string> ResolvePackageName { get; set; } = (s) =>
    {
        if (s.EndsWith("_msgs") || s.EndsWith("_interfaces") ||
            s.EndsWith("_msg") || s.EndsWith("_interface") ||
            s.EndsWith("_srvs") || s.EndsWith("_srv"))
        {
            return string.Join("_", s.Split('_')[..^1]).ToPascalCase();
        }

        return s.ToPascalCase();
    };

    public Func<ServiceBuildContext, ServiceMetadata, string> ResolveServiceClassName { get; set; } = (ctx, s) => ctx.Type switch
    {
        ServiceType.Plain => s.Name + "Service",
        ServiceType.ActionSendGoal => ((ActionBuildContext)ctx.ParentContext!).ClassName + "SendGoalService",
        ServiceType.ActionGetResult => ((ActionBuildContext)ctx.ParentContext!).ClassName + "GetResultService",
        _ => throw new NotImplementedException()
    };

    public Func<ActionBuildContext, ActionMetadata, string> ResolveActionClassName { get; set; } = (ctx, s) => s.Name + "Action";

    public Func<MessageBuildContext, ComplexTypeMetadata, string> ResolveMessageClassName { get; set; } = (ctx, metadata)
        =>
        metadata is MessageMetadata
        ? ctx.Type switch
        {
            MessageType.Plain => metadata.Name,
            MessageType.ServiceRequest => ((ServiceBuildContext)ctx.ParentContext!).ClassName + "Request",
            MessageType.ServiceResponse => ((ServiceBuildContext)ctx.ParentContext!).ClassName + "Response",
            MessageType.ActionFeedback => ((ActionBuildContext)ctx.ParentContext!).ClassName + "Feedback",
            MessageType.ActionGoal => ((ActionBuildContext)ctx.ParentContext!).ClassName + "Goal",
            MessageType.ActionResult => ((ActionBuildContext)ctx.ParentContext!).ClassName + "Result",
            MessageType.ActionFeedbackMessage => ((ActionBuildContext)ctx.ParentContext!).ClassName + "FeedbackMessage",
            _ => throw new NotImplementedException()
        }
        : metadata.Name;

    public Func<MessageBuildContext, ComplexTypeMetadata, string> ResolveMessagePrivStructName { get; set; } = (ctx, metadata) => "Priv";

    public Func<MessageBuildContext, ComplexTypeMetadata, string> ResolveMessagePrivStructSequenceName { get; set; } = (ctx, metadata) => "PrivSequence";

    public Func<MessageBuildContext, FieldMetadata, string> ResolveFieldName { get; set; } = (ctx, field) =>
    {
        if (field is ConstantFieldMetadata)
        {
            return field.Name;
        }

        var fieldName = field.Name.ToPascalCase();
        return (ctx.ClassName == fieldName ||
                ctx.PrivStructName == fieldName ||
                ctx.PrivStructSequenceName == fieldName) ? fieldName + "_" : fieldName;
    };

}
