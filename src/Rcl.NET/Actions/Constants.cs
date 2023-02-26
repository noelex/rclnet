using Rcl.Qos;

namespace Rcl.Actions;

internal static class Constants
{
    public const string StatusTopic = "/_action/status";
    public const string FeedbackTopic = "/_action/feedback";
    public const string SendGoalService = "/_action/send_goal";
    public const string CancelGoalService = "/_action/cancel_goal";
    public const string GetResultService = "/_action/get_result";
}
