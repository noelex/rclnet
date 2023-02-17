using Rcl;
using Rcl.Graph;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("graph_monitor");

_ = Task.Run(async () =>
{
    await foreach (var item in node.Graph.ReadEventsAsync())
    {
        switch (item)
        {
            case NodeEstablishedEvent nodeEstablished:
                Console.WriteLine("NodeEstablished: " + nodeEstablished.Node.Name);
                break;
            case NodeRemovedEvent nodeRemoved:
                Console.WriteLine("NodeRemoved: " + nodeRemoved.Node.Name);
                break;
            case TopicEstablishedEvent topicEstablished:
                Console.WriteLine("TopicEstablished: " + topicEstablished.Topic.Name);
                break;
            case TopicRemovedEvent topicRemoved:
                Console.WriteLine("TopicRemoved: " + topicRemoved.Topic.Name);
                break;
            case ServiceEstablishedEvent serviceEstablished:
                Console.WriteLine("ServiceEstablished: " + serviceEstablished.Service.Name);
                break;
            case ServiceRemovedEvent serviceRemoved:
                Console.WriteLine("ServiceRemoved: " + serviceRemoved.Service.Name);
                break;
            case PublisherEstablishedEvent publisherEstablished:
                Console.WriteLine("PublisherEstablished: " + publisherEstablished.Publisher.Topic.Name +
                 ", Type = " + publisherEstablished.Publisher.Type + ", Node = " + publisherEstablished.Publisher.Node);
                break;
            case PublisherRemovedEvent publisherRemoved:
                Console.WriteLine("PublisherRemoved: " + publisherRemoved.Publisher.Topic.Name +
                 ", Type = " + publisherRemoved.Publisher.Type + ", Node = " + publisherRemoved.Publisher.Node);
                break;
            case SubscriberEstablishedEvent subscriberEstablished:
                Console.WriteLine("SubscriberEstablished: " + subscriberEstablished.Subscriber.Topic.Name +
                 ", Type = " + subscriberEstablished.Subscriber.Type + ", Node = " + subscriberEstablished.Subscriber.Node);
                break;
            case SubscriberRemovedEvent subscriberRemoved:
                Console.WriteLine("SubscriberRemoved: " + subscriberRemoved.Subscriber.Topic.Name +
                 ", Type = " + subscriberRemoved.Subscriber.Type + ", Node = " + subscriberRemoved.Subscriber.Node);
                break;
            case ServerEstablishedEvent serverEstablished:
                Console.WriteLine("ServerEstablished: " + serverEstablished.Server.Service.Name +
                ", Type = " + serverEstablished.Server.Type + ", Node = " + serverEstablished.Server.Node);
                break;
            case ServerRemovedEvent serverRemoved:
                Console.WriteLine("ServerRemoved: " + serverRemoved.Server.Service.Name +
                ", Type = " + serverRemoved.Server.Type + ", Node = " + serverRemoved.Server.Node);
                break;
            case ClientEstablishedEvent clientEstablished:
                Console.WriteLine("ClientEstablished: " + clientEstablished.Client.Service.Name +
                ", Type = " + clientEstablished.Client.Type + ", Node = " + clientEstablished.Client.Node);
                break;
            case ClientRemovedEvent clientRemoved:
                Console.WriteLine("ClientRemoved: " + clientRemoved.Client.Service.Name + ", Type = " + clientRemoved.Client.Type);
                break;
            case ActionEstablishedEvent serviceEstablished:
                Console.WriteLine("ActionEstablished: " + serviceEstablished.Action.Name);
                break;
            case ActionRemovedEvent serviceRemoved:
                Console.WriteLine("ActionRemoved: " + serviceRemoved.Action.Name);
                break;
            case ActionServerEstablishedEvent serverEstablished:
                Console.WriteLine("ActionServerEstablished: " + serverEstablished.ActionServer.Action.Name +
                ", Type = " + serverEstablished.ActionServer.Type + ", Node = " + serverEstablished.ActionServer.Node);
                break;
            case ActionServerRemovedEvent serverRemoved:
                Console.WriteLine("ActionServerRemoved: " + serverRemoved.ActionServer.Action.Name +
                ", Type = " + serverRemoved.ActionServer.Type + ", Node = " + serverRemoved.ActionServer.Node);
                break;
            case ActionClientEstablishedEvent clientEstablished:
                Console.WriteLine("ActionClientEstablished: " + clientEstablished.ActionClient.Action.Name +
                ", Type = " + clientEstablished.ActionClient.Type + ", Node = " + clientEstablished.ActionClient.Node);
                break;
            case ActionClientRemovedEvent clientRemoved:
                Console.WriteLine("ActionClientRemoved: " + clientRemoved.ActionClient.Action.Name + ", Type = " + clientRemoved.ActionClient.Type);
                break;
        }
    }
});

Console.ReadLine();