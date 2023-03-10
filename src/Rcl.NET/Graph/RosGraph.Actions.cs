using Microsoft.Toolkit.HighPerformance.Buffers;
using Rosidl.Runtime.Interop;
using System.Collections.Concurrent;

namespace Rcl.Graph;

public partial class RosGraph : IGraphBuilder
{
    private const string ActionFeedbackTopicSuffix = "/_action/feedback";
    private const string ActionFeedbackTypeSuffix = "_FeedbackMessage";

    private readonly ConcurrentDictionary<string, RosAction> _actions = new();

    private readonly List<RosAction> _newActions = new(), _removedActions = new();

    private readonly List<RosActionEndPoint>
        _totalActionClients = new(), _newActionClients = new(), _removedActionClients = new(),
        _totalActionServers = new(), _newActionServers = new(), _removedActionServers = new();

    private void BuildActions()
    {
        foreach (var node in _nodes.Values)
        {
            using var sp = SpanOwner<NameWithType>.Allocate(Math.Max(node.Publishers.Count, node.Subscribers.Count));
            var count = GetActionEndpointsByNode(node, sp.Span, true);
            node.UpdateActionServers(this, sp.Span.Slice(0, count));

            count = GetActionEndpointsByNode(node, sp.Span, false);
            node.UpdateActionClients(this, sp.Span.Slice(0, count));
        }

        var offset = 0;
        using var temp = SpanOwner<RosActionEndPoint>.Allocate(
            Math.Max(_totalActionClients.Count, _totalActionServers.Count));

        foreach (var action in _actions.Values)
        {
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

        foreach (var (k, v) in _actions)
        {
            if (v.Clients.Count == 0 && v.Servers.Count == 0)
            {
                _actions.Remove(k, out _);
                _removedActions.Add(v);
            }
        }
    }

    private static int GetActionEndpointsByNode(RosNode node, Span<NameWithType> items, bool publishers)
    {
        var i = 0;
        var target = publishers ? node.Publishers : node.Subscribers;
        foreach (var sub in target)
        {
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
        return i;
    }

    void IGraphBuilder.OnAddActionServer(RosActionEndPoint endpoint)
    {
        _newActionServers.Add(endpoint);
    }

    void IGraphBuilder.OnRemoveActionServer(RosActionEndPoint endpoint)
    {
        _removedActionServers.Add(endpoint);
    }

    void IGraphBuilder.OnEnumerateActionServer(RosActionEndPoint endpoint)
    {
        _totalActionServers.Add(endpoint);
    }

    void IGraphBuilder.OnAddActionClient(RosActionEndPoint endpoint)
    {
        _newActionClients.Add(endpoint);
    }

    void IGraphBuilder.OnRemoveActionClient(RosActionEndPoint endpoint)
    {
        _removedActionClients.Add(endpoint);
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
            _newActions.Add(s);
        }
        return s;
    }
}
