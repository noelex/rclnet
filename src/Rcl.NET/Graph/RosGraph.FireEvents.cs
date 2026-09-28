namespace Rcl.Graph;

public partial class RosGraph
{
    private void PublishEvent(RosGraphEvent e, ref List<Exception>? errors)
    {
        var handlers = Volatile.Read(ref _handlerSnapshot);

        try
        {
            while (_observersEnumerator.MoveNext())
            {
                try
                {
                    _observersEnumerator.Current.Value.OnNext(e);
                }
                catch (Exception error)
                {
                    (errors ??= new()).Add(error);
                }
            }

            foreach (GraphChangedEventHandler handler in handlers)
            {
                try
                {
                    handler(e);
                }
                catch (Exception error)
                {
                    (errors ??= new()).Add(error);
                }
            }
        }
        finally
        {
            _observersEnumerator.Reset();
        }
    }

    private void PublishEvents<T, TFactory>(
        Dictionary<T, UpdateOp> updates,
        UpdateOp operation,
        TFactory factory,
        ref List<Exception>? errors)
        where T : notnull
        where TFactory : IEventFactory<T>
    {
        foreach (var (k, v) in updates)
        {
            if (v == operation)
            {
                PublishEvent(operation == UpdateOp.Add
                    ? factory.CreateAppeared(this, k)
                    : factory.CreateDisappeared(this, k), ref errors);
            }
        }
    }

    private void FireEvents()
    {
        if (_observers.IsEmpty && Volatile.Read(ref _handlerSnapshot).Length == 0)
        {
            return;
        }

        List<Exception>? errors = null;
        PublishEvents(_nodeUpdates, UpdateOp.Add, new NodeEventFactory(), ref errors);
        PublishEvents(_topicUpdates, UpdateOp.Add, new TopicEventFactory(), ref errors);
        PublishEvents(_publisherUpdates, UpdateOp.Add, new TopicEndPointEventFactory(isPublisher: true), ref errors);
        PublishEvents(_subscriberUpdates, UpdateOp.Add, new TopicEndPointEventFactory(isPublisher: false), ref errors);
        PublishEvents(_serviceUpdates, UpdateOp.Add, new ServiceEventFactory(), ref errors);
        PublishEvents(_serverUpdates, UpdateOp.Add, new ServiceEndPointEventFactory(isServer: true), ref errors);
        PublishEvents(_clientUpdates, UpdateOp.Add, new ServiceEndPointEventFactory(isServer: false), ref errors);
        PublishEvents(_actionUpdates, UpdateOp.Add, new ActionEventFactory(), ref errors);
        PublishEvents(_actionServerUpdates, UpdateOp.Add, new ActionEndPointEventFactory(isServer: true), ref errors);
        PublishEvents(_actionClientUpdates, UpdateOp.Add, new ActionEndPointEventFactory(isServer: false), ref errors);

        // Removals reverse the precedence levels, preserving endpoint order within each level.
        PublishEvents(_actionServerUpdates, UpdateOp.Remove, new ActionEndPointEventFactory(isServer: true), ref errors);
        PublishEvents(_actionClientUpdates, UpdateOp.Remove, new ActionEndPointEventFactory(isServer: false), ref errors);
        PublishEvents(_actionUpdates, UpdateOp.Remove, new ActionEventFactory(), ref errors);
        PublishEvents(_serverUpdates, UpdateOp.Remove, new ServiceEndPointEventFactory(isServer: true), ref errors);
        PublishEvents(_clientUpdates, UpdateOp.Remove, new ServiceEndPointEventFactory(isServer: false), ref errors);
        PublishEvents(_serviceUpdates, UpdateOp.Remove, new ServiceEventFactory(), ref errors);
        PublishEvents(_publisherUpdates, UpdateOp.Remove, new TopicEndPointEventFactory(isPublisher: true), ref errors);
        PublishEvents(_subscriberUpdates, UpdateOp.Remove, new TopicEndPointEventFactory(isPublisher: false), ref errors);
        PublishEvents(_topicUpdates, UpdateOp.Remove, new TopicEventFactory(), ref errors);
        PublishEvents(_nodeUpdates, UpdateOp.Remove, new NodeEventFactory(), ref errors);

        if (errors != null)
        {
            throw new GraphEventDispatchException(errors);
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
internal sealed class GraphEventDispatchException(IEnumerable<Exception> errors)
    : AggregateException("ROS graph callbacks failed after the complete event batch was dispatched.", errors)
{
}
