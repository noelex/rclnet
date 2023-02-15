using CppAst.CodeGen.CSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Generator.CSharp.Helpers;
internal static class Extensions
{
    public static void AddTypeSupportAttribute(this CSharpClass cls, string name)
    {
        cls.Attributes.Add(new CSharpFreeAttribute($"global::Rosidl.Runtime.TypeSupportAttribute(\"{name}\")"));
    }

    public static void AddTypeSupportInterface(this CSharpClass cls, MessageBuildContext context)
    {
        cls.BaseTypes.Add(new CSharpFreeType(
            context.Type switch
            {
                MessageType.Plain => "global::Rosidl.Runtime.IMessage",
                MessageType.ServiceRequest => "global::Rosidl.Runtime.IServiceRequest",
                MessageType.ServiceResponse => "global::Rosidl.Runtime.IServiceResponse",
                MessageType.ActionGoal => "global::Rosidl.Runtime.IActionGoal",
                MessageType.ActionFeedback => "global::Rosidl.Runtime.IActionFeedback",
                MessageType.ActionResult => "global::Rosidl.Runtime.IActionResult",
                MessageType.ActionFeedbackMessage => "global::Rosidl.Runtime.IActionFeedbackMessage",
                _ => throw new NotSupportedException()
            }
        ));
    }
}
