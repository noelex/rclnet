namespace Rcl.Graph;

internal interface IGraphBuilder
{
    RosService GetOrAddService(string name);

    void OnAddActionServer(RosActionEndPoint endpoint);

    void OnRemoveActionServer(RosActionEndPoint endpoint);

    void OnAddActionClient(RosActionEndPoint endpoint);

    void OnRemoveActionClient(RosActionEndPoint endpoint);

    RosAction GetOrAddAction(string name);

    void OnAddServiceServer(RosServiceEndPoint server);

    void OnRemoveServiceServer(RosServiceEndPoint server);

    void OnAddServiceClient(RosServiceEndPoint client);

    void OnRemoveServiceClient(RosServiceEndPoint client);

    void OnAddPublisher(RosTopicEndPoint publisher);

    void OnRemovePublisher(RosTopicEndPoint publisher);

    void OnAddSubscriber(RosTopicEndPoint subscriber);

    void OnRemoveSubscriber(RosTopicEndPoint subscriber);
}