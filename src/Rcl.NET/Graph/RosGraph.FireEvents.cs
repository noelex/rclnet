namespace Rcl.Graph;

public partial class RosGraph
{
    private void FireEvents()
    {
        foreach (var node in _newNodes)
        {
            var e = new NodeEstablishedEvent(this, node);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newTopics)
        {
            var e = new TopicEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newPublishers)
        {
            var e = new PublisherEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newSubscribers)
        {
            var e = new SubscriberEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedPublishers)
        {
            var e = new PublisherRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedSubscribers)
        {
            var e = new SubscriberRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedTopics)
        {
            var e = new TopicRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newServices)
        {
            var e = new ServiceEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newServers)
        {
            var e = new ServerEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newClients)
        {
            var e = new ClientEstablishedEvent(this, item);
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
            var e = new ServerRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedClients)
        {
            var e = new ClientRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedServices)
        {
            var e = new ServiceRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedNodes)
        {

            var e = new NodeRemovedEvent(this, item);
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
            var e = new ActionEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newActionServers)
        {
            var e = new ActionServerEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _newActionClients)
        {
            var e = new ActionClientEstablishedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedActionServers)
        {
            var e = new ActionServerRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedActionClients)
        {
            var e = new ActionClientRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }

        foreach (var item in _removedActions)
        {
            var e = new ActionRemovedEvent(this, item);
            foreach (var obs in _observers.Values)
            {
                obs.OnNext(e);
            }
            _channel.Writer.TryWrite(e);
            GraphChanged?.Invoke(e);
        }
    }
}