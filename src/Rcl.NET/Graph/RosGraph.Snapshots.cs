namespace Rcl.Graph;

[Flags]
internal enum SnapshotChanges
{
    Publishers = 1,
    Subscribers = 2,
    Servers = 4,
    Clients = 8,
    ActionServers = 16,
    ActionClients = 32
}

public partial class RosGraph
{
    private bool _nodesChanged, _topicsChanged, _servicesChanged, _actionsChanged;
    private readonly Dictionary<RosNode, SnapshotChanges> _changedNodes = new();
    private readonly Dictionary<RosTopic, SnapshotChanges> _changedTopics = new();
    private readonly Dictionary<RosService, SnapshotChanges> _changedServices = new();
    private readonly Dictionary<RosAction, SnapshotChanges> _changedActions = new();

    private void TrackSnapshotChange<T>(T item) where T : class
    {
        switch (item)
        {
            case RosNode:
                _nodesChanged = true;
                break;
            case RosTopic:
                _topicsChanged = true;
                break;
            case RosService:
                _servicesChanged = true;
                break;
            case RosAction:
                _actionsChanged = true;
                break;
            case RosTopicEndPoint endpoint:
                var topicChanges = endpoint.EndPointType == TopicEndPointType.Publisher
                    ? SnapshotChanges.Publishers : SnapshotChanges.Subscribers;
                MarkChanged(_changedNodes, endpoint.Node, topicChanges);
                MarkChanged(_changedTopics, endpoint.Topic, topicChanges);
                break;
            case RosServiceEndPoint endpoint:
                var serviceChanges = endpoint.EndPointType == ServiceEndPointType.Server
                    ? SnapshotChanges.Servers : SnapshotChanges.Clients;
                MarkChanged(_changedNodes, endpoint.Node, serviceChanges);
                MarkChanged(_changedServices, endpoint.Service, serviceChanges);
                break;
            case RosActionEndPoint endpoint:
                var isServer = endpoint.EndPointType == ActionEndPointType.Server;
                MarkChanged(_changedNodes, endpoint.Node,
                    isServer ? SnapshotChanges.ActionServers : SnapshotChanges.ActionClients);
                MarkChanged(_changedActions, endpoint.Action,
                    isServer ? SnapshotChanges.Servers : SnapshotChanges.Clients);
                break;
        }
    }

    private static void MarkChanged<T>(Dictionary<T, SnapshotChanges> target, T item, SnapshotChanges changes)
        where T : notnull
    {
        target.TryGetValue(item, out var pending);
        target[item] = pending | changes;
    }

    private void PublishSnapshots()
    {
        // Include removed objects so retained references and disappearance events see their final state.
        foreach (var (node, changes) in _changedNodes)
        {
            node.PublishSnapshots(changes);
        }

        foreach (var (topic, changes) in _changedTopics)
        {
            topic.PublishSnapshots(changes);
        }

        foreach (var (service, changes) in _changedServices)
        {
            service.PublishSnapshots(changes);
        }

        foreach (var (action, changes) in _changedActions)
        {
            action.PublishSnapshots(changes);
        }

        if (_nodesChanged)
        {
            Volatile.Write(ref _nodesSnapshot, (IReadOnlyCollection<RosNode>)_nodes.Values);
        }

        if (_topicsChanged)
        {
            Volatile.Write(ref _topicsSnapshot, (IReadOnlyCollection<RosTopic>)_topics.Values);
        }

        if (_servicesChanged)
        {
            Volatile.Write(ref _servicesSnapshot, (IReadOnlyCollection<RosService>)_services.Values);
        }

        if (_actionsChanged)
        {
            Volatile.Write(ref _actionsSnapshot, (IReadOnlyCollection<RosAction>)_actions.Values);
        }

        // Keep these pending across failed builds, just like the staged events.
        _changedNodes.Clear();
        _changedTopics.Clear();
        _changedServices.Clear();
        _changedActions.Clear();
        _nodesChanged = _topicsChanged = _servicesChanged = _actionsChanged = false;
    }
}
