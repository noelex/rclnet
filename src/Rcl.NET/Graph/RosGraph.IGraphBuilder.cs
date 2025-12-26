using System.Buffers;
using System.Runtime.InteropServices;

namespace Rcl.Graph;

public partial class RosGraph
{
    RosService IGraphBuilder.GetOrAddService(string name)
    {
        if (!_services.TryGetValue(name, out var s))
        {
            _services[name] = s = new(name);
            _newServices.Add(s);
        }
        return s;
    }

    void IGraphBuilder.OnAddServiceServer(RosServiceEndPoint server)
    {
        _newServers.Add(server);
    }

    void IGraphBuilder.OnRemoveServiceServer(RosServiceEndPoint server)
    {
        _removedServers.Add(server);
    }

    void IGraphBuilder.OnEnumerateServiceServer(RosServiceEndPoint server)
    {
        ref var endpoints = ref CollectionsMarshal.GetValueRefOrAddDefault(_totalServers, server.Service, out _);
        endpoints ??= new();
        endpoints.Add(server);
    }

    void IGraphBuilder.OnAddServiceClient(RosServiceEndPoint client)
    {
        _newClients.Add(client);
    }

    void IGraphBuilder.OnRemoveServiceClient(RosServiceEndPoint client)
    {
        _removedClients.Add(client);
    }

    void IGraphBuilder.OnEnumerateServiceClient(RosServiceEndPoint client)
    {
        ref var endpoints = ref CollectionsMarshal.GetValueRefOrAddDefault(_totalClients, client.Service, out _);
        endpoints ??= new();
        endpoints.Add(client);
    }

    void IGraphBuilder.OnAddPublisher(RosTopicEndPoint publisher)
    {
        _newPublishers.Add(publisher);
    }

    void IGraphBuilder.OnRemovePublisher(RosTopicEndPoint publisher)
    {
        _removedPublishers.Add(publisher);
    }

    void IGraphBuilder.OnEnumeratePublisher(RosTopicEndPoint publisher)
    {
        ref var endpoints = ref CollectionsMarshal.GetValueRefOrAddDefault(_totalPublications, publisher.Node, out _);
        endpoints ??= new();
        endpoints.Add(publisher);
    }

    void IGraphBuilder.OnAddSubscriber(RosTopicEndPoint subscriber)
    {
        _newSubscribers.Add(subscriber);
    }

    void IGraphBuilder.OnRemoveSubscriber(RosTopicEndPoint subscriber)
    {
        _removedSubscribers.Add(subscriber);
    }

    void IGraphBuilder.OnEnumerateSubscriber(RosTopicEndPoint subscriber)
    {
        ref var endpoints = ref CollectionsMarshal.GetValueRefOrAddDefault(_totalSubscriptions, subscriber.Node, out _);
        endpoints ??= new();
        endpoints.Add(subscriber);
    }

    private class PoolingList<T> : IDisposable
    {
        private T[] _data;
        private int _count = 0;

        public PoolingList(int capacity = 8)
        {
            _data = ArrayPool<T>.Shared.Rent(capacity);
        }

        public void Add(T item)
        {
            if (_count >= _data.Length)
            {
                var newData = ArrayPool<T>.Shared.Rent(_data.Length * 2);
                Array.Copy(_data, newData, _data.Length);
                ArrayPool<T>.Shared.Return(_data);
                _data = newData;
            }
            _data[_count++] = item;
        }

        public Span<T> AsSpan() => _data.AsSpan(0, _count);

        public void Dispose()
        {
            ArrayPool<T>.Shared.Return(_data);
        }
    }
}