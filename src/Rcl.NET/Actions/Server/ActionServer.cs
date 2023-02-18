using Rcl.Internal.Services;
using Rcl.Introspection;
using Rcl.Qos;
using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Text;

namespace Rcl.Actions.Server;

internal class ActionServer : IRclObject
{
    private readonly RclNodeImpl _node;
    private readonly Encoding _textEncoding;
    private readonly ActionIntrospection _typesupport;

    private readonly IRclService _sendGoalService, _getResultService, _cancelGoalService;
    private readonly IRclPublisher _statusPublisher, _feedbackPublisher;

    private readonly INativeActionGoalHandler _handler;
    private readonly DynamicFunctionTable _functions;

    private readonly MessageIntrospection _goalIntrospection, _feedbackIntrospection, _resultIntrospection;

    public ActionServer(RclNodeImpl node, string actionName,
        string typesupportName, TypeSupportHandle actionTypesupport, INativeActionGoalHandler handler, Encoding textEncoding)
    {
        _node = node;
        _handler = handler;
        _textEncoding = textEncoding;

        var statusTopicName = actionName + Constants.StatusTopic;
        var feedbackTopicName = actionName + Constants.FeedbackTopic;
        var sendGoalServiceName = actionName + Constants.SendGoalService;
        var cancelGoalServiceName = actionName + Constants.CancelGoalService;
        var getResultServiceName = actionName + Constants.GetResultService;

        _typesupport = new ActionIntrospection(actionTypesupport);
        _functions = new DynamicFunctionTable(typesupportName);

        var done = false;
        try
        {
            _sendGoalService = new IntrospectionService(node,
                sendGoalServiceName, _typesupport.GoalServiceTypeSupport,
                new DelegateNativeServiceCallHandler(SendGoalHandler, this), QosProfile.ServicesDefault);
            _getResultService = new IntrospectionService(node,
                getResultServiceName, _typesupport.ResultServiceTypeSupport,
                new DelegateNativeServiceCallHandler(GetResultHandler, this), QosProfile.ServicesDefault);
            _cancelGoalService = _node.CreateNativeService<CancelGoalService>(
                cancelGoalServiceName, CancelGoalHandler, QosProfile.ServicesDefault, this);

            _statusPublisher = _node.CreatePublisher<GoalStatusArray>(statusTopicName, Constants.StatusQoS, textEncoding);
            _feedbackPublisher = new RclNativePublisher(node, feedbackTopicName, _typesupport.FeedbackMessageTypeSupport, QosProfile.SensorData);

            // In case the given action name gets normalized.
            var sep = _feedbackPublisher.Name!.LastIndexOf(Constants.FeedbackTopic);
            Name = _feedbackPublisher.Name.Substring(0, sep);
            done = true;
        }
        finally
        {
            if (!done)
            {
                _feedbackPublisher?.Dispose();
                _statusPublisher?.Dispose();
                _cancelGoalService?.Dispose();
                _getResultService?.Dispose();
                _sendGoalService?.Dispose();
            }
        }
    }

    public string Name { get; }

    private static unsafe void SendGoalHandler(RosMessageBuffer request, RosMessageBuffer response, object? state)
    {
        var self = (ActionServer)state!;

        var goalId = self._typesupport.GoalService.Request.AsRef<UUID.Priv>(request.Data, 0);
        var goal = self._typesupport.GoalService.Request.GetMemberPointer(request.Data, 1);

        if (self._handler.CanAccept(goalId, new RosMessageBuffer(goal, static (a, b) => { })))
        {
            var copiedGoal = new RosMessageBuffer(
                self._functions.CreateGoal(),
                (p, tab) => ((DynamicFunctionTable)tab!).DestroyGoal(p),
                self._functions);

    
        }
    }

    private static void GetResultHandler(RosMessageBuffer request, RosMessageBuffer response, object? state)
    {

    }

    private static void CancelGoalHandler(RosMessageBuffer request, RosMessageBuffer response, object? state)
    {

    }

    public void Dispose()
    {
        throw new NotImplementedException();
    }
}
