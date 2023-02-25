namespace Rcl.Graph;

public partial class RosGraph
{
    private void FireEvents()
    {
        foreach (var node in _newNodes)
        {
            var e = new NodeAppearedEvent(this, node);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newTopics)
        {
            var e = new TopicAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newPublishers)
        {
            var e = new PublisherAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newSubscribers)
        {
            var e = new SubscriberAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedPublishers)
        {
            var e = new PublisherDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedSubscribers)
        {
            var e = new SubscriberDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedTopics)
        {
            var e = new TopicDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newServices)
        {
            var e = new ServiceAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newServers)
        {
            var e = new ServerAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newClients)
        {
            var e = new ClientAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        FireActionEvents();

        foreach (var item in _removedServers)
        {
            var e = new ServerDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedClients)
        {
            var e = new ClientDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedServices)
        {
            var e = new ServiceDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedNodes)
        {

            var e = new NodeDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }
    }

    private void FireActionEvents()
    {
        foreach (var item in _newActions)
        {
            var e = new ActionAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newActionServers)
        {
            var e = new ActionServerAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newActionClients)
        {
            var e = new ActionClientAppearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedActionServers)
        {
            var e = new ActionServerDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedActionClients)
        {
            var e = new ActionClientDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedActions)
        {
            var e = new ActionDisappearedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }
    }
}