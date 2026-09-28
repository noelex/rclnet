using Rcl.Graph;
using Rcl.Qos;
using System.Collections;
using System.Reflection;

namespace Rcl.NET.Tests;

public class GraphEventDispatchTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task MixedChangesPreserveEventOrder(bool observer)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var owner = context.CreateNode(NameGenerator.GenerateNodeName());
        var graph = new RosGraph((Rcl.Internal.RclNodeImpl)owner, static _ => true);
        var dispatch = PrepareChanges(graph);
        var events = new List<Type>();
        using var subscription = observer ? graph.Subscribe(new EventObserver(events)) : null;

        if (!observer)
        {
            graph.GraphChanged += e => events.Add(e.GetType());
        }

        dispatch();

        Assert.Equal(new[]
        {
            typeof(NodeAppearedEvent), typeof(TopicAppearedEvent),
            typeof(PublisherAppearedEvent), typeof(SubscriberAppearedEvent),
            typeof(ServiceAppearedEvent), typeof(ServerAppearedEvent), typeof(ClientAppearedEvent),
            typeof(ActionAppearedEvent), typeof(ActionServerAppearedEvent), typeof(ActionClientAppearedEvent),
            typeof(ActionServerDisappearedEvent), typeof(ActionClientDisappearedEvent), typeof(ActionDisappearedEvent),
            typeof(ServerDisappearedEvent), typeof(ClientDisappearedEvent), typeof(ServiceDisappearedEvent),
            typeof(PublisherDisappearedEvent), typeof(SubscriberDisappearedEvent),
            typeof(TopicDisappearedEvent), typeof(NodeDisappearedEvent)
        }, events);
    }

    [Fact]
    public async Task ChangesWithoutListenersDoNotAllocateEvents()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var owner = context.CreateNode(NameGenerator.GenerateNodeName());
        var graph = new RosGraph((Rcl.Internal.RclNodeImpl)owner, static _ => true);
        var dispatch = PrepareChanges(graph);

        for (int i = 0; i < 100; i++)
        {
            dispatch();
        }

        var before = GC.GetAllocatedBytesForCurrentThread();

        for (int i = 0; i < 100; i++)
        {
            dispatch();
        }

        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    private static Action PrepareChanges(RosGraph graph)
    {
        // Stage a mixed batch directly so DDS discovery cannot split or reorder the test changes.
        for (int operation = 2; operation >= 1; operation--)
        {
            var node = new RosNode(new NodeName($"node{operation}", "/"), "/");
            var topic = new RosTopic($"/topic{operation}");
            var service = new RosService($"/service{operation}");
            var action = new RosAction($"/action{operation}");
            Stage("_nodeUpdates", node, operation);
            Stage("_topicUpdates", topic, operation);
            Stage("_publisherUpdates", new RosTopicEndPoint(default, topic, node,
                TopicEndPointType.Publisher, "test/msg/Message", QosProfile.Default), operation);
            Stage("_subscriberUpdates", new RosTopicEndPoint(default, topic, node,
                TopicEndPointType.Subscriber, "test/msg/Message", QosProfile.Default), operation);
            Stage("_serviceUpdates", service, operation);
            Stage("_serverUpdates", new RosServiceEndPoint(node, ServiceEndPointType.Server, service, "test/srv/Service"), operation);
            Stage("_clientUpdates", new RosServiceEndPoint(node, ServiceEndPointType.Client, service, "test/srv/Service"), operation);
            Stage("_actionUpdates", action, operation);
            Stage("_actionServerUpdates", new RosActionEndPoint(node, ActionEndPointType.Server, action, "test/action/Action"), operation);
            Stage("_actionClientUpdates", new RosActionEndPoint(node, ActionEndPointType.Client, action, "test/action/Action"), operation);
        }

        return typeof(RosGraph).GetMethod("FireEvents", BindingFlags.Instance | BindingFlags.NonPublic)!
            .CreateDelegate<Action>(graph);

        void Stage(string fieldName, object item, int operation)
        {
            var field = typeof(RosGraph).GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic)!;
            var updates = (IDictionary)field.GetValue(graph)!;
            updates.Add(item, Enum.ToObject(field.FieldType.GenericTypeArguments[1], operation));
        }
    }

    private sealed class EventObserver(List<Type> events) : IObserver<RosGraphEvent>
    {
        public void OnNext(RosGraphEvent value)
        {
            events.Add(value.GetType());
        }

        public void OnCompleted()
        {
        }

        public void OnError(Exception error)
        {
            throw error;
        }
    }
}
