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

    private readonly List<RosActionEndPoint>
        _totalActionClients = new(), _totalActionServers = new();

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

        int offset;
        using var temp = SpanOwner<RosActionEndPoint>.Allocate(
            Math.Max(_totalActionClients.Count, _totalActionServers.Count));

        try
        {
            while (_actionsEnumerator.MoveNext())
            {
                var action = _actionsEnumerator.Current.Value;
                offset = 0;
                foreach (var client in _totalActionClients)
                {
                    if (client.Action == action)
                    {
                        temp.Span[offset++] = client;
                    }
                }
                action.ResetClients(temp.Span.Slice(0, offset));

                offset = 0;
                foreach (var server in _totalActionServers)
                {
                    if (server.Action == action)
                    {
                        temp.Span[offset++] = server;
                    }
                }
                action.ResetServers(temp.Span.Slice(0, offset));
            }
        }
        finally
        {
            _actionsEnumerator.Reset();
        }

        try
        {
            while (_actionsEnumerator.MoveNext())
            {
                var (k, v) = _actionsEnumerator.Current;
                if (v.ClientCount == 0 && v.ServerCount == 0)
                {
                    _actions.Remove(k, out _);
                    OnRemove(_actionUpdates, v);
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
    }

    void IGraphBuilder.OnRemoveActionServer(RosActionEndPoint endpoint)
    {
        OnRemove(_actionServerUpdates, endpoint);
    }

    void IGraphBuilder.OnEnumerateActionServer(RosActionEndPoint endpoint)
    {
        _totalActionServers.Add(endpoint);
    }

    void IGraphBuilder.OnAddActionClient(RosActionEndPoint endpoint)
    {
        OnAdd(_actionClientUpdates, endpoint);
    }

    void IGraphBuilder.OnRemoveActionClient(RosActionEndPoint endpoint)
    {
        OnRemove(_actionClientUpdates, endpoint);
    }

    void IGraphBuilder.OnEnumerateActionClient(RosActionEndPoint endpoint)
    {
        _totalActionClients.Add(endpoint);
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
