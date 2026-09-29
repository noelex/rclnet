namespace Rcl.Graph;

public partial class RosGraph
{
    private readonly SnapshotPublisher _snapshotPublisher = new();

    private void TrackSnapshotChange<T>(T item, UpdateOp operation) where T : class
    {
        var adding = operation == UpdateOp.Add;
        switch (item)
        {
            case RosNode node:
                _nodesSnapshot.Stage(node, adding);
                break;
            case RosTopic topic:
                _topicsSnapshot.Stage(topic, adding);
                break;
            case RosService service:
                _servicesSnapshot.Stage(service, adding);
                break;
            case RosAction action:
                _actionsSnapshot.Stage(action, adding);
                break;
            case RosTopicEndPoint endpoint:
                if (endpoint.EndPointType == TopicEndPointType.Publisher)
                {
                    endpoint.Node.PublishersSnapshot.Stage(endpoint, adding);
                    endpoint.Topic.PublishersSnapshot.Stage(endpoint, adding);
                }
                else
                {
                    endpoint.Node.SubscribersSnapshot.Stage(endpoint, adding);
                    endpoint.Topic.SubscribersSnapshot.Stage(endpoint, adding);
                }

                break;
            case RosServiceEndPoint endpoint:
                if (endpoint.EndPointType == ServiceEndPointType.Server)
                {
                    endpoint.Node.ServersSnapshot.Stage(endpoint, adding);
                    endpoint.Service.ServersSnapshot.Stage(endpoint, adding);
                }
                else
                {
                    endpoint.Node.ClientsSnapshot.Stage(endpoint, adding);
                    endpoint.Service.ClientsSnapshot.Stage(endpoint, adding);
                }

                break;
            case RosActionEndPoint endpoint:
                if (endpoint.EndPointType == ActionEndPointType.Server)
                {
                    endpoint.Node.ActionServersSnapshot.Stage(endpoint, adding);
                    endpoint.Action.ServersSnapshot.Stage(endpoint, adding);
                }
                else
                {
                    endpoint.Node.ActionClientsSnapshot.Stage(endpoint, adding);
                    endpoint.Action.ClientsSnapshot.Stage(endpoint, adding);
                }

                break;
        }
    }
}
