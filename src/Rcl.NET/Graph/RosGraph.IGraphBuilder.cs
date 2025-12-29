namespace Rcl.Graph;

public partial class RosGraph
{
    RosService IGraphBuilder.GetOrAddService(string name)
    {
        if (!_services.TryGetValue(name, out var s))
        {
            _services[name] = s = new(name);
            OnAdd(_serviceUpdates, s);
        }
        return s;
    }

    void IGraphBuilder.OnAddServiceServer(RosServiceEndPoint server)
    {
        OnAdd(_serverUpdates, server);
        server.Service.AddServer(server);
    }

    void IGraphBuilder.OnRemoveServiceServer(RosServiceEndPoint server)
    {
        OnRemove(_serverUpdates, server);
        server.Service.RemoveServer(server);
    }

    void IGraphBuilder.OnAddServiceClient(RosServiceEndPoint client)
    {
        OnAdd(_clientUpdates, client);
        client.Service.AddClient(client);
    }

    void IGraphBuilder.OnRemoveServiceClient(RosServiceEndPoint client)
    {
        OnRemove(_clientUpdates, client);
        client.Service.RemoveClient(client);
    }

    void IGraphBuilder.OnAddPublisher(RosTopicEndPoint publisher)
    {
        OnAdd(_publisherUpdates, publisher);
        publisher.Node.AddPublisher(publisher);
    }

    void IGraphBuilder.OnRemovePublisher(RosTopicEndPoint publisher)
    {
        OnRemove(_publisherUpdates, publisher);
        publisher.Node.RemovePublisher(publisher);
    }

    void IGraphBuilder.OnAddSubscriber(RosTopicEndPoint subscriber)
    {
        OnAdd(_subscriberUpdates, subscriber);
        subscriber.Node.AddSubscriber(subscriber);
    }

    void IGraphBuilder.OnRemoveSubscriber(RosTopicEndPoint subscriber)
    {
        OnRemove(_subscriberUpdates, subscriber);
        subscriber.Node.RemoveSubscriber(subscriber);
    }
}