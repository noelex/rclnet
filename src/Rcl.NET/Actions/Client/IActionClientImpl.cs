using Rcl.Internal.Clients;
using Rcl.Introspection;
using Rosidl.Messages.Action;

namespace Rcl.Actions.Client;

interface IActionClientImpl
{
    ActionClientOptions Options { get; }

    bool TryRemoveGoal(Guid goalId);

    ActionIntrospection Introspection { get; }

    IRclClient<CancelGoalServiceRequest, CancelGoalServiceResponse> CancelClient { get; }

    IntrospectionClient GetResultClient { get; }

    MessageBufferHelper BufferHelper { get; }
}
