using Microsoft.Toolkit.HighPerformance.Buffers;
using Rosidl.Runtime.Interop;
using System.Collections.Concurrent;

namespace Rcl.Graph;

public partial class RosGraph : IGraphBuilder
{
    private const string ActionFeedbackTopicSuffix = "/_action/feedback";
    private const string ActionFeedbackTypeSuffix = "_FeedbackMessage";

    private readonly ConcurrentDictionary<string, RosAction> _actions = new();

    private readonly Dictionary<RosAction, UpdateOp> _actionUpdates = new();

    private readonly Dictionary<RosActionEndPoint, UpdateOp>
        _actionServerUpdates = new(), _actionClientUpdates = new();

    private readonly IEnumerator<KeyValuePair<string, RosAction>> _actionsEnumerator;

    private void BuildActions()
    {
        try
        {
            while (_nodesEnumerator.MoveNext())
            {
                var node = _nodesEnumerator.Current.Value;
                using var sp = SpanOwner<NameWithType>.Allocate(Math.Max(node.PublisherCount, node.SubscriberCount));
                var count = GetActionEndpointsByNode(node, sp.Span, true);
                node.UpdateActionServers(this, sp.Span.Slice(0, count));

                count = GetActionEndpointsByNode(node, sp.Span, false);
                node.UpdateActionClients(this, sp.Span.Slice(0, count));
            }
        }
        finally
        {
            _nodesEnumerator.Reset();
        }

        try
        {
            while (_actionsEnumerator.MoveNext())
            {
                var (k, action) = _actionsEnumerator.Current;
                if (action.ClientCount == 0 && action.ServerCount == 0)
                {
                    _actions.Remove(k, out _);
                    OnRemove(_actionUpdates, action);
                }
            }
        }
        finally
        {
            _actionsEnumerator.Reset();
        }
    }

    private static int GetActionEndpointsByNode(RosNode node, Span<NameWithType> items, bool publishers)
    {
        var i = 0;
        var target = publishers ? node.PublishersEnumerator : node.SubscribersEnumerator;
        try
        {
            while (target.MoveNext())
            {
                var sub = target.Current.Key;
                var nameidx = sub.Topic.Name.LastIndexOf(ActionFeedbackTopicSuffix);
                var typeidx = sub.Type.LastIndexOf(ActionFeedbackTypeSuffix);

                if (nameidx > 0 && nameidx + ActionFeedbackTopicSuffix.Length == sub.Topic.Name.Length &&
                    typeidx > 0 && typeidx + ActionFeedbackTypeSuffix.Length == sub.Type.Length)
                {
                    var actionName = StringMarshal.CreatePooledString(sub.Topic.Name.AsSpan().Slice(0, nameidx));
                    var typeName = StringMarshal.CreatePooledString(sub.Type.AsSpan().Slice(0, typeidx));

                    items[i++] = new(actionName, typeName);
                }
            }
        }
        finally
        {
            target.Reset();
        }
        return i;
    }

    void IGraphBuilder.OnAddActionServer(RosActionEndPoint endpoint)
    {
        OnAdd(_actionServerUpdates, endpoint);
        endpoint.Action.AddServer(endpoint);
    }

    void IGraphBuilder.OnRemoveActionServer(RosActionEndPoint endpoint)
    {
        OnRemove(_actionServerUpdates, endpoint);
        endpoint.Action.RemoveServer(endpoint);
    }

    void IGraphBuilder.OnAddActionClient(RosActionEndPoint endpoint)
    {
        OnAdd(_actionClientUpdates, endpoint);
        endpoint.Action.AddClient(endpoint);
    }

    void IGraphBuilder.OnRemoveActionClient(RosActionEndPoint endpoint)
    {
        OnRemove(_actionClientUpdates, endpoint);
        endpoint.Action.RemoveClient(endpoint);
    }

    RosAction IGraphBuilder.GetOrAddAction(string name)
    {
        if (!_actions.TryGetValue(name, out var s))
        {
            _actions[name] = s = new(name);
            OnAdd(_actionUpdates, s);
        }
        return s;
    }
}
