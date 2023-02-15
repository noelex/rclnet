using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace Rcl.Introspection;

internal class ActionIntrospection
{
    public unsafe ActionIntrospection(TypeSupportHandle typeSupport)
    {
        var ts=typeSupport.GetActionTypeSupport();

        CancelServiceTypeSupport = new(new(ts->CancelService), HandleType.Service);
        GoalServiceTypeSupport = new(new(ts->GoalService), HandleType.Service);
        ResultServiceTypeSupport = new(new(ts->ResultService), HandleType.Service);
        FeedbackMessageTypeSupport = new(new(ts->FeedbackMessage), HandleType.Message);
        StatusMessageTypeSupport = new(new(ts->StatusMessage), HandleType.Message);

        CancelService = new (CancelServiceTypeSupport);
        GoalService = new(GoalServiceTypeSupport);
        ResultService = new(ResultServiceTypeSupport);
        FeedbackMessage = new (FeedbackMessageTypeSupport);
        StatusMessage = new(StatusMessageTypeSupport);
    }

    public TypeSupportHandle CancelServiceTypeSupport { get; }

    public TypeSupportHandle GoalServiceTypeSupport { get; }

    public TypeSupportHandle ResultServiceTypeSupport { get; }

    public TypeSupportHandle FeedbackMessageTypeSupport { get; }

    public TypeSupportHandle StatusMessageTypeSupport { get; }


    public ServiceIntrospection CancelService { get; }

    public ServiceIntrospection GoalService { get; }

    public ServiceIntrospection ResultService { get; }

    public MessageIntrospection FeedbackMessage { get; }

    public MessageIntrospection StatusMessage { get; }
}
