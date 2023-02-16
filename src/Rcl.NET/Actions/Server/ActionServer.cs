using Rcl.Internal.Services;
using Rcl.Introspection;
using Rcl.Qos;
using Rosidl.Messages.Action;
using Rosidl.Runtime;
using System.Text;

namespace Rcl.Actions.Server;

internal class ActionServer<TAction, TGoal, TResult, TFeedback> : IRclObject
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TResult : IActionResult
        where TFeedback : IActionFeedback
{
    private readonly RclNodeImpl _node;
    private readonly Encoding _textEncoding;
    private readonly ActionIntrospection _typesupport;

    private readonly IRclService _sendGoalService, _getResultService, _cancelGoalService;
    private readonly IRclPublisher _statusPublisher, _feedbackPublisher;

    public ActionServer(RclNodeImpl node, string actionName, Encoding textEncoding)
    {
        _node = node;
        _textEncoding = textEncoding;

        var statusTopicName = actionName + Constants.StatusTopic;
        var feedbackTopicName = actionName + Constants.FeedbackTopic;
        var sendGoalServiceName = actionName + Constants.SendGoalService;
        var cancelGoalServiceName = actionName + Constants.CancelGoalService;
        var getResultServiceName = actionName + Constants.GetResultService;

        _typesupport = new ActionIntrospection(TAction.GetTypeSupportHandle());

        var done = false;
        try
        {
            _sendGoalService = new IntrospectionService(node,
                sendGoalServiceName, _typesupport.GoalServiceTypeSupport,
                new DelegateNativeServiceCallHandler(SendGoalHandler, this), QosProfile.ServicesDefault);
            _getResultService = new IntrospectionService(node,
                getResultServiceName, _typesupport.ResultServiceTypeSupport,
                new DelegateNativeServiceCallHandler(GetResultHandler, this), QosProfile.ServicesDefault);
            _cancelGoalService = _node.CreateNativeService<CancelGoalService, CancelGoalServiceRequest, CancelGoalServiceResponse>(
                cancelGoalServiceName, CancelGoalHandler, QosProfile.ServicesDefault, this);

            _statusPublisher = _node.CreatePublisher<GoalStatusArray>(statusTopicName, Constants.StatusQoS, textEncoding);
            _feedbackPublisher = new RclNativePublisher(node, feedbackTopicName, _typesupport.FeedbackMessageTypeSupport, QosProfile.SensorData);

            // In case the given action name gets normalized.
            var sep = _feedbackPublisher.Name!.LastIndexOf("/_action/feedback");
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

    private static void SendGoalHandler(RosMessageBuffer request, RosMessageBuffer response, object? state)
    {
        
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
