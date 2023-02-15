using Rcl.Internal.Clients;
using Rcl.Introspection;
using Rosidl.Messages.Action;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Actions.Client;

interface IActionClientImpl
{
    bool TryRemoveGoal(Guid goalId);

    ActionIntrospection Introspection { get; }

    IRclClient<CancelGoalServiceRequest, CancelGoalServiceResponse> CancelClient { get; }

    IntrospectionRclClient GetResultClient { get; }

    DynamicFunctionTable Functions { get; }
}
