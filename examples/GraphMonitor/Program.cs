using Rcl;
using Rcl.Graph;
using Rcl.Logging;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("graph_monitor");

_ = Task.Run(async () =>
{
    await foreach (var item in node.Graph.ReadEventsAsync())
    {
        switch (item)
        {
            case NodeEstablishedEvent nodeEstablished:
                node.Logger.LogInformation("NodeEstablished: " + nodeEstablished.Node.Name);
                break;
            case NodeRemovedEvent nodeRemoved:
                node.Logger.LogInformation("NodeRemoved: " + nodeRemoved.Node.Name);
                break;
            case TopicEstablishedEvent topicEstablished:
                node.Logger.LogInformation("TopicEstablished: " + topicEstablished.Topic.Name);
                break;
            case TopicRemovedEvent topicRemoved:
                node.Logger.LogInformation("TopicRemoved: " + topicRemoved.Topic.Name);
                break;
            case ServiceEstablishedEvent serviceEstablished:
                node.Logger.LogInformation("ServiceEstablished: " + serviceEstablished.Service.Name);
                break;
            case ServiceRemovedEvent serviceRemoved:
                node.Logger.LogInformation("ServiceRemoved: " + serviceRemoved.Service.Name);
                break;
            case PublisherEstablishedEvent publisherEstablished:
                node.Logger.LogInformation("PublisherEstablished: " + publisherEstablished.Publisher.Topic.Name +
                 ", Type = " + publisherEstablished.Publisher.Type + ", Node = " + publisherEstablished.Publisher.Node);
                break;
            case PublisherRemovedEvent publisherRemoved:
                node.Logger.LogInformation("PublisherRemoved: " + publisherRemoved.Publisher.Topic.Name +
                 ", Type = " + publisherRemoved.Publisher.Type + ", Node = " + publisherRemoved.Publisher.Node);
                break;
            case SubscriberEstablishedEvent subscriberEstablished:
                node.Logger.LogInformation("SubscriberEstablished: " + subscriberEstablished.Subscriber.Topic.Name +
                 ", Type = " + subscriberEstablished.Subscriber.Type + ", Node = " + subscriberEstablished.Subscriber.Node);
                break;
            case SubscriberRemovedEvent subscriberRemoved:
                node.Logger.LogInformation("SubscriberRemoved: " + subscriberRemoved.Subscriber.Topic.Name +
                 ", Type = " + subscriberRemoved.Subscriber.Type + ", Node = " + subscriberRemoved.Subscriber.Node);
                break;
            case ServerEstablishedEvent serverEstablished:
                node.Logger.LogInformation("ServerEstablished: " + serverEstablished.Server.Service.Name +
                ", Type = " + serverEstablished.Server.Type + ", Node = " + serverEstablished.Server.Node);
                break;
            case ServerRemovedEvent serverRemoved:
                node.Logger.LogInformation("ServerRemoved: " + serverRemoved.Server.Service.Name +
                ", Type = " + serverRemoved.Server.Type + ", Node = " + serverRemoved.Server.Node);
                break;
            case ClientEstablishedEvent clientEstablished:
                node.Logger.LogInformation("ClientEstablished: " + clientEstablished.Client.Service.Name +
                ", Type = " + clientEstablished.Client.Type + ", Node = " + clientEstablished.Client.Node);
                break;
            case ClientRemovedEvent clientRemoved:
                node.Logger.LogInformation("ClientRemoved: " + clientRemoved.Client.Service.Name + ", Type = " + clientRemoved.Client.Type);
                break;
            case ActionEstablishedEvent serviceEstablished:
                node.Logger.LogInformation("ActionEstablished: " + serviceEstablished.Action.Name);
                break;
            case ActionRemovedEvent serviceRemoved:
                node.Logger.LogInformation("ActionRemoved: " + serviceRemoved.Action.Name);
                break;
            case ActionServerEstablishedEvent serverEstablished:
                node.Logger.LogInformation("ActionServerEstablished: " + serverEstablished.ActionServer.Action.Name +
                ", Type = " + serverEstablished.ActionServer.Type + ", Node = " + serverEstablished.ActionServer.Node);
                break;
            case ActionServerRemovedEvent serverRemoved:
                node.Logger.LogInformation("ActionServerRemoved: " + serverRemoved.ActionServer.Action.Name +
                ", Type = " + serverRemoved.ActionServer.Type + ", Node = " + serverRemoved.ActionServer.Node);
                break;
            case ActionClientEstablishedEvent clientEstablished:
                node.Logger.LogInformation("ActionClientEstablished: " + clientEstablished.ActionClient.Action.Name +
                ", Type = " + clientEstablished.ActionClient.Type + ", Node = " + clientEstablished.ActionClient.Node);
                break;
            case ActionClientRemovedEvent clientRemoved:
                node.Logger.LogInformation("ActionClientRemoved: " + clientRemoved.ActionClient.Action.Name + ", Type = " + clientRemoved.ActionClient.Type);
                break;
        }
    }
});

Console.ReadLine();