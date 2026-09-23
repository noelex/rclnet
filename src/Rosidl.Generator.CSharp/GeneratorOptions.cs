using Rosidl.Generator.CSharp.Builders;

namespace Rosidl.Generator.CSharp;

/// <summary>
/// Specifies which ROSIDL native ABI layout the generated code targets.
/// </summary>
public enum RosidlAbiMode
{
    /// <summary>
    /// Generates the ROSIDL ABI used through ROS 2 Kilted.
    /// </summary>
    V1,

    /// <summary>
    /// Generates the ROSIDL ABI introduced in ROS 2 Lyrical.
    /// </summary>
    V2,

    /// <summary>
    /// Generates both layouts and selects the active ABI at runtime.
    /// </summary>
    Portable,

    /// <summary>
    /// Generates the layout used by the ROS distribution selected through
    /// the <c>ROS_DISTRO</c> environment variable.
    /// </summary>
    Native
}

public class GeneratorOptions
{
    public GeneratorOptions()
    {
        ResolveNamespace = _ => RootNamespace;
    }

    public string RootNamespace { get; set; } = "Rosidl.Messages";

    private RosidlAbiMode _abi = RosidlAbiMode.Portable;

    /// <summary>
    /// Gets or sets the ROSIDL native ABI mode used for code generation.
    /// </summary>
    /// <remarks>
    /// Setting this property to <see cref="RosidlAbiMode.Native"/> resolves the current
    /// <c>ROS_DISTRO</c> immediately and stores the corresponding concrete ABI mode.
    /// </remarks>
    public RosidlAbiMode Abi
    {
        get => _abi;
        set => _abi = value == RosidlAbiMode.Native
            ? ResolveNativeAbi(Environment.GetEnvironmentVariable("ROS_DISTRO"))
            : value;
    }

    public Func<string, string> ResolveNamespace { get; set; }

    private static RosidlAbiMode ResolveNativeAbi(string? distro)
    {
        return distro switch
        {
            "foxy" or "humble" or "iron" or "jazzy" or "kilted" => RosidlAbiMode.V1,
            "lyrical" => RosidlAbiMode.V2,
            _ => throw new NotSupportedException(
                $"ABI mode 'native' requires a supported ROS distribution in ROS_DISTRO, " +
                $"but its current value is '{FormatDistribution(distro)}'. " +
                "Supported distributions: foxy, humble, iron, jazzy, kilted, lyrical.")
        };
    }

    private static string FormatDistribution(string? distro) => distro switch
    {
        null => "<unset>",
        "" => "<empty>",
        _ => distro
    };

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
        ServiceType.Plain => s.Id.Name + "Service",
        ServiceType.ActionSendGoal => ((ActionBuildContext)ctx.ParentContext!).ClassName + "SendGoalService",
        ServiceType.ActionGetResult => ((ActionBuildContext)ctx.ParentContext!).ClassName + "GetResultService",
        _ => throw new NotImplementedException()
    };

    public Func<ActionBuildContext, ActionMetadata, string> ResolveActionClassName { get; set; } = (ctx, s) => s.Id.Name + "Action";

    public Func<MessageBuildContext, ComplexTypeMetadata, string> ResolveMessageClassName { get; set; } = (ctx, metadata) =>
        {
            // Special case for service events as they refer Request / Response classes with private names.
            // We need to convert those private names to public ones.
            if (metadata is not MessageMetadata)
            {
                if (ctx.Type == MessageType.ServiceEvent && ctx.ParentContext is ServiceBuildContext)
                {
                    var parentContext = (ServiceBuildContext)ctx.ParentContext!;

                    if (metadata.Id.Package == parentContext.Metadata.Id.Package
                        && metadata.Id.SubFolder == parentContext.Metadata.Id.SubFolder
                        && metadata.Id.Name.StartsWith(parentContext.Metadata.Id.Name))
                    {
                        if (metadata.Id.Name.LastIndexOf("_Request") == parentContext.Metadata.Id.Name.Length)
                        {
                            return parentContext.ClassName + "Request";
                        }
                        else if (metadata.Id.Name.LastIndexOf("_Response") == parentContext.Metadata.Id.Name.Length)
                        {
                            return parentContext.ClassName + "Response";
                        }
                    }
                }

                if (ctx.Type == MessageType.ActionFeedbackMessage &&
                   ctx.ParentContext is ActionBuildContext)
                {
                    var parentContext = (ActionBuildContext)ctx.ParentContext!;
                    if (metadata.Id.Package == parentContext.Metadata.Id.Package
                        && metadata.Id.SubFolder == parentContext.Metadata.Id.SubFolder
                        && metadata.Id.Name.StartsWith(parentContext.Metadata.Id.Name))
                    {
                        if (metadata.Id.Name.LastIndexOf("_Feedback") == parentContext.Metadata.Id.Name.Length)
                        {
                            return parentContext.ClassName + "Feedback";
                        }
                    }
                }

                if (ctx.ParentContext is ServiceBuildContext serviceContext &&
                    serviceContext.ParentContext is ActionBuildContext actionContext)
                {
                    var parentContext = actionContext;
                    if (metadata.Id.Package == parentContext.Metadata.Id.Package
                        && metadata.Id.SubFolder == parentContext.Metadata.Id.SubFolder
                        && metadata.Id.Name.StartsWith(parentContext.Metadata.Id.Name))
                    {
                        if (ctx.Type == MessageType.ServiceResponse)
                        {
                            if (metadata.Id.Name.LastIndexOf("_Result") == parentContext.Metadata.Id.Name.Length)
                            {
                                return parentContext.ClassName + "Result";
                            }
                        }
                        else if (ctx.Type == MessageType.ServiceRequest)
                        {
                            if (metadata.Id.Name.LastIndexOf("_Goal") == parentContext.Metadata.Id.Name.Length)
                            {
                                return parentContext.ClassName + "Goal";
                            }
                        }
                    }
                }
            }

            return metadata is MessageMetadata
            ? ctx.Type switch
            {
                MessageType.Plain => metadata.Id.Name,
                MessageType.ServiceRequest => ((ServiceBuildContext)ctx.ParentContext!).ClassName + "Request",
                MessageType.ServiceResponse => ((ServiceBuildContext)ctx.ParentContext!).ClassName + "Response",
                MessageType.ServiceEvent => ((ServiceBuildContext)ctx.ParentContext!).ClassName + "Event",
                MessageType.ActionFeedback => ((ActionBuildContext)ctx.ParentContext!).ClassName + "Feedback",
                MessageType.ActionGoal => ((ActionBuildContext)ctx.ParentContext!).ClassName + "Goal",
                MessageType.ActionResult => ((ActionBuildContext)ctx.ParentContext!).ClassName + "Result",
                MessageType.ActionFeedbackMessage => ((ActionBuildContext)ctx.ParentContext!).ClassName + "FeedbackMessage",
                _ => throw new NotImplementedException()
            }
            : metadata.Id.Name;
        };

    public Func<MessageBuildContext, ComplexTypeMetadata, string> ResolveMessagePrivStructName { get; set; } = (ctx, metadata) => "Priv";

    public Func<MessageBuildContext, ComplexTypeMetadata, string> ResolveMessagePrivStructSequenceName { get; set; } = (ctx, metadata) => "PrivSequence";

    public Func<MessageBuildContext, FieldMetadata, string> ResolveFieldName { get; set; } = (ctx, field) =>
    {
        if (field is ConstantFieldMetadata)
        {
            return field.Name;
        }

        var fieldName = field.Name.ToPascalCase();
        return ctx.GetNormalizedFieldName(fieldName);
    };

}
