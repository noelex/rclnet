namespace Rcl.Graph;

public partial class RosGraph
{
    private void PublishEvent(RosGraphEvent e)
    {
        try
        {
            while (_observersEnumerator.MoveNext())
            {
                _observersEnumerator.Current.Value.OnNext(e);
            }
            GraphChanged?.Invoke(e);
        }
        finally
        {
            _observersEnumerator.Reset();
        }
    }

    private void AddEvents<T, TFactory>(
        ref PoolingList<IndexedEvent> events,
        int precedence,
        Dictionary<T, UpdateOp> updates,
        TFactory factory
       )
        where T : notnull
        where TFactory : IEventFactory<T>
    {
        foreach (var (k, v) in updates)
        {
            if (v == UpdateOp.Add)
            {
                events.Add(new(events.Count, precedence, factory.CreateAppeared(this, k)));
            }
            else
            {
                events.Add(new(events.Count, 10000 - precedence, factory.CreateDisappeared(this, k)));
            }
        }
    }

    record struct IndexedEvent(int Index, int Class, RosGraphEvent Event) : IComparable<IndexedEvent>
    {
        public readonly int CompareTo(IndexedEvent other)
        {
            var classCompare = Class.CompareTo(other.Class);
            return classCompare == 0 ? Index.CompareTo(other.Index) : classCompare;
        }
    }

    private void FireEvents()
    {
        var events = new PoolingList<IndexedEvent>();
        try
        {
            AddEvents(ref events, 0, _nodeUpdates, new NodeEventFactory());
            AddEvents(ref events, 1, _topicUpdates, new TopicEventFactory());
            AddEvents(ref events, 2, _publisherUpdates, new TopicEndPointEventFactory(isPublisher: true));
            AddEvents(ref events, 2, _subscriberUpdates, new TopicEndPointEventFactory(isPublisher: false));
            AddEvents(ref events, 3, _serviceUpdates, new ServiceEventFactory());
            AddEvents(ref events, 4, _serverUpdates, new ServiceEndPointEventFactory(isServer: true));
            AddEvents(ref events, 4, _clientUpdates, new ServiceEndPointEventFactory(isServer: false));
            AddEvents(ref events, 5, _actionUpdates, new ActionEventFactory());
            AddEvents(ref events, 6, _actionServerUpdates, new ActionEndPointEventFactory(isServer: true));
            AddEvents(ref events, 6, _actionClientUpdates, new ActionEndPointEventFactory(isServer: false));

            var eventSpan = events.AsSpan();
            eventSpan.Sort();
            foreach (var e in eventSpan)
            {
                PublishEvent(e.Event);
            }
        }
        finally
        {
            events.Dispose();
        }
    }

    interface IEventFactory<TArg>
    {
        RosGraphEvent CreateAppeared(RosGraph sender, TArg arg);

        RosGraphEvent CreateDisappeared(RosGraph sender, TArg arg);
    }

    readonly struct NodeEventFactory : IEventFactory<RosNode>
    {
        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosNode arg)
            => new NodeAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosNode arg)
            => new NodeDisappearedEvent(sender, arg);
    }

    readonly struct TopicEventFactory : IEventFactory<RosTopic>
    {
        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosTopic arg)
            => new TopicAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosTopic arg)
            => new TopicDisappearedEvent(sender, arg);
    }

    readonly struct TopicEndPointEventFactory : IEventFactory<RosTopicEndPoint>
    {
        private readonly bool _publisher;

        public TopicEndPointEventFactory(bool isPublisher) => _publisher = isPublisher;

        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosTopicEndPoint arg)
            => _publisher ? new PublisherAppearedEvent(sender, arg) : new SubscriberAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosTopicEndPoint arg)
            => _publisher ? new PublisherDisappearedEvent(sender, arg) : new SubscriberDisappearedEvent(sender, arg);
    }

    readonly struct ServiceEventFactory : IEventFactory<RosService>
    {
        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosService arg)
            => new ServiceAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosService arg)
            => new ServiceDisappearedEvent(sender, arg);
    }

    readonly struct ServiceEndPointEventFactory : IEventFactory<RosServiceEndPoint>
    {
        private readonly bool _server;

        public ServiceEndPointEventFactory(bool isServer) => _server = isServer;

        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosServiceEndPoint arg)
            => _server ? new ServerAppearedEvent(sender, arg) : new ClientAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosServiceEndPoint arg)
            => _server ? new ServerDisappearedEvent(sender, arg) : new ClientDisappearedEvent(sender, arg);
    }

    readonly struct ActionEventFactory : IEventFactory<RosAction>
    {
        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosAction arg)
            => new ActionAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosAction arg)
            => new ActionDisappearedEvent(sender, arg);
    }

    readonly struct ActionEndPointEventFactory : IEventFactory<RosActionEndPoint>
    {
        private readonly bool _server;

        public ActionEndPointEventFactory(bool isServer) => _server = isServer;

        public readonly RosGraphEvent CreateAppeared(RosGraph sender, RosActionEndPoint arg)
            => _server ? new ActionServerAppearedEvent(sender, arg) : new ActionClientAppearedEvent(sender, arg);

        public readonly RosGraphEvent CreateDisappeared(RosGraph sender, RosActionEndPoint arg)
            => _server ? new ActionServerDisappearedEvent(sender, arg) : new ActionClientDisappearedEvent(sender, arg);
    }
}