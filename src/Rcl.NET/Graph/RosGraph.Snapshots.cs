namespace Rcl.Graph;

public partial class RosGraph
{
    private bool _nodesChanged, _topicsChanged, _servicesChanged, _actionsChanged;
    private readonly HashSet<RosNode> _changedNodes = new();
    private readonly HashSet<RosTopic> _changedTopics = new();
    private readonly HashSet<RosService> _changedServices = new();
    private readonly HashSet<RosAction> _changedActions = new();

    private void TrackSnapshotChange<T>(T item) where T : class
    {
        switch (item)
        {
            case RosNode node:
                _nodesChanged = true;
                _changedNodes.Add(node);
                break;
            case RosTopic topic:
                _topicsChanged = true;
                _changedTopics.Add(topic);
                break;
            case RosService service:
                _servicesChanged = true;
                _changedServices.Add(service);
                break;
            case RosAction action:
                _actionsChanged = true;
                _changedActions.Add(action);
                break;
            case RosTopicEndPoint endpoint:
                _changedNodes.Add(endpoint.Node);
                _changedTopics.Add(endpoint.Topic);
                break;
            case RosServiceEndPoint endpoint:
                _changedNodes.Add(endpoint.Node);
                _changedServices.Add(endpoint.Service);
                break;
            case RosActionEndPoint endpoint:
                _changedNodes.Add(endpoint.Node);
                _changedActions.Add(endpoint.Action);
                break;
        }
    }

    private void PublishSnapshots()
    {
        // Include removed objects so retained references and disappearance events see their final state.
        foreach (var node in _changedNodes)
        {
            node.PublishSnapshots();
        }

        foreach (var topic in _changedTopics)
        {
            topic.PublishSnapshots();
        }

        foreach (var service in _changedServices)
        {
            service.PublishSnapshots();
        }

        foreach (var action in _changedActions)
        {
            action.PublishSnapshots();
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

        // Keep these pending across failed builds; event staging is cleared independently.
        _changedNodes.Clear();
        _changedTopics.Clear();
        _changedServices.Clear();
        _changedActions.Clear();
        _nodesChanged = _topicsChanged = _servicesChanged = _actionsChanged = false;
    }
}
