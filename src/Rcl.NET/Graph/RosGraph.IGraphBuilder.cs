namespace Rcl.Graph;

public partial class RosGraph
{
    RosService IGraphBuilder.GetOrAddService(string name)
    {
        if (!_services.TryGetValue(name, out var s))
        {
            _services[name] = s = new(name);
            _newServices.Add(s);
        }
        return s;
    }

    void IGraphBuilder.OnAddServiceServer(RosServiceEndPoint server)
    {
        _newServers.Add(server);
    }

    void IGraphBuilder.OnRemoveServiceServer(RosServiceEndPoint server)
    {
        _removedServers.Add(server);
    }

    void IGraphBuilder.OnEnumerateServiceServer(RosServiceEndPoint server)
    {
        _totalServers.Add(server);
    }

    void IGraphBuilder.OnAddServiceClient(RosServiceEndPoint client)
    {
        _newClients.Add(client);
    }

    void IGraphBuilder.OnRemoveServiceClient(RosServiceEndPoint client)
    {
        _removedClients.Add(client);
    }

    void IGraphBuilder.OnEnumerateServiceClient(RosServiceEndPoint client)
    {
        _totalClients.Add(client);
    }

    void IGraphBuilder.OnAddPublisher(RosTopicEndPoint publisher)
    {
        _newPublishers.Add(publisher);
    }

    void IGraphBuilder.OnRemovePublisher(RosTopicEndPoint publisher)
    {
        _removedPublishers.Add(publisher);
    }

    void IGraphBuilder.OnEnumeratePublisher(RosTopicEndPoint publisher)
    {
        _totalPublications.Add(publisher);
    }

    void IGraphBuilder.OnAddSubscriber(RosTopicEndPoint subscriber)
    {
        _newSubscribers.Add(subscriber);
    }

    void IGraphBuilder.OnRemoveSubscriber(RosTopicEndPoint subscriber)
    {
        _removedSubscribers.Add(subscriber);
    }

    void IGraphBuilder.OnEnumerateSubscriber(RosTopicEndPoint subscriber)
    {
        _totalSubscriptions.Add(subscriber);
    }
}