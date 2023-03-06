using Rcl;
using Rcl.Graph;
using Rcl.Logging;

await using var ctx = new RclContext(args);
using var node = ctx.CreateNode("graph_monitor");

_ = Task.Run(async () =>
{
    await foreach (var item in node.Graph.ReadEventsAsync())
    {
        switch (item)
        {
            case NodeAppearedEvent nodeEstablished:
                node.Logger.LogInformation("NodeAppeared: " + nodeEstablished.Node.Name);
                break;
            case NodeDisappearedEvent nodeRemoved:
                node.Logger.LogInformation("NodeDisappeared: " + nodeRemoved.Node.Name);
                break;
            case TopicAppearedEvent topicEstablished:
                node.Logger.LogInformation("TopicAppeared: " + topicEstablished.Topic.Name);
                break;
            case TopicDisappearedEvent topicRemoved:
                node.Logger.LogInformation("TopicDisappeared: " + topicRemoved.Topic.Name);
                break;
            case ServiceAppearedEvent serviceEstablished:
                node.Logger.LogInformation("ServiceAppeared: " + serviceEstablished.Service.Name);
                break;
            case ServiceDisappearedEvent serviceRemoved:
                node.Logger.LogInformation("ServiceDisappeared: " + serviceRemoved.Service.Name);
                break;
            case PublisherAppearedEvent publisherEstablished:
                node.Logger.LogInformation("PublisherAppeared: " + publisherEstablished.Publisher.Topic.Name +
                 ", Type = " + publisherEstablished.Publisher.Type + ", Node = " + publisherEstablished.Publisher.Node);
                break;
            case PublisherDisappearedEvent publisherRemoved:
                node.Logger.LogInformation("PublisherDisappeared: " + publisherRemoved.Publisher.Topic.Name +
                 ", Type = " + publisherRemoved.Publisher.Type + ", Node = " + publisherRemoved.Publisher.Node);
                break;
            case SubscriberAppearedEvent subscriberEstablished:
                node.Logger.LogInformation("SubscriberAppeared: " + subscriberEstablished.Subscriber.Topic.Name +
                 ", Type = " + subscriberEstablished.Subscriber.Type + ", Node = " + subscriberEstablished.Subscriber.Node);
                break;
            case SubscriberDisappearedEvent subscriberRemoved:
                node.Logger.LogInformation("SubscriberDisappeared: " + subscriberRemoved.Subscriber.Topic.Name +
                 ", Type = " + subscriberRemoved.Subscriber.Type + ", Node = " + subscriberRemoved.Subscriber.Node);
                break;
            case ServerAppearedEvent serverEstablished:
                node.Logger.LogInformation("ServerAppeared: " + serverEstablished.Server.Service.Name +
                ", Type = " + serverEstablished.Server.Type + ", Node = " + serverEstablished.Server.Node);
                break;
            case ServerDisappearedEvent serverRemoved:
                node.Logger.LogInformation("ServerDisappeared: " + serverRemoved.Server.Service.Name +
                ", Type = " + serverRemoved.Server.Type + ", Node = " + serverRemoved.Server.Node);
                break;
            case ClientAppearedEvent clientEstablished:
                node.Logger.LogInformation("ClientAppeared: " + clientEstablished.Client.Service.Name +
                ", Type = " + clientEstablished.Client.Type + ", Node = " + clientEstablished.Client.Node);
                break;
            case ClientDisappearedEvent clientRemoved:
                node.Logger.LogInformation("ClientDisappeared: " + clientRemoved.Client.Service.Name + ", Type = " + clientRemoved.Client.Type);
                break;
            case ActionAppearedEvent serviceEstablished:
                node.Logger.LogInformation("ActionAppeared: " + serviceEstablished.Action.Name);
                break;
            case ActionDisappearedEvent serviceRemoved:
                node.Logger.LogInformation("ActionDisappeared: " + serviceRemoved.Action.Name);
                break;
            case ActionServerAppearedEvent serverEstablished:
                node.Logger.LogInformation("ActionServerAppeared: " + serverEstablished.ActionServer.Action.Name +
                ", Type = " + serverEstablished.ActionServer.Type + ", Node = " + serverEstablished.ActionServer.Node);
                break;
            case ActionServerDisappearedEvent serverRemoved:
                node.Logger.LogInformation("ActionServerDisappeared: " + serverRemoved.ActionServer.Action.Name +
                ", Type = " + serverRemoved.ActionServer.Type + ", Node = " + serverRemoved.ActionServer.Node);
                break;
            case ActionClientAppearedEvent clientEstablished:
                node.Logger.LogInformation("ActionClientAppeared: " + clientEstablished.ActionClient.Action.Name +
                ", Type = " + clientEstablished.ActionClient.Type + ", Node = " + clientEstablished.ActionClient.Node);
                break;
            case ActionClientDisappearedEvent clientRemoved:
                node.Logger.LogInformation("ActionClientDisappeared: " + clientRemoved.ActionClient.Action.Name + ", Type = " + clientRemoved.ActionClient.Type);
                break;
        }
    }
});

Console.ReadLine();