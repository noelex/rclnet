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
            OnAdd(_serviceUpdates, s);
        }
        return s;
    }

    void IGraphBuilder.OnAddServiceServer(RosServiceEndPoint server)
    {
        OnAdd(_serverUpdates, server);
    }

    void IGraphBuilder.OnRemoveServiceServer(RosServiceEndPoint server)
    {
        OnRemove(_serverUpdates, server);
    }

    void IGraphBuilder.OnEnumerateServiceServer(RosServiceEndPoint server)
    {
        ref var endpoints = ref CollectionsMarshal.GetValueRefOrAddDefault(_totalServers, server.Service, out var found);
        if (!found) endpoints = new();
        endpoints.Add(server);
    }

    void IGraphBuilder.OnAddServiceClient(RosServiceEndPoint client)
    {
        OnAdd(_clientUpdates, client);
    }

    void IGraphBuilder.OnRemoveServiceClient(RosServiceEndPoint client)
    {
        OnRemove(_clientUpdates, client);
    }

    void IGraphBuilder.OnEnumerateServiceClient(RosServiceEndPoint client)
    {
        ref var endpoints = ref CollectionsMarshal.GetValueRefOrAddDefault(_totalClients, client.Service, out var found);
        if (!found) endpoints = new();
        endpoints.Add(client);
    }

    void IGraphBuilder.OnAddPublisher(RosTopicEndPoint publisher)
    {
        OnAdd(_publisherUpdates, publisher);
        publisher.Node.AddPublisher(publisher);
    }

    void IGraphBuilder.OnRemovePublisher(RosTopicEndPoint publisher)
    {
        OnRemove(_publisherUpdates, publisher);
        publisher.Node.RemovePublisher(publisher);
    }

    void IGraphBuilder.OnAddSubscriber(RosTopicEndPoint subscriber)
    {
        OnAdd(_subscriberUpdates, subscriber);
        subscriber.Node.AddSubscriber(subscriber);
    }

    void IGraphBuilder.OnRemoveSubscriber(RosTopicEndPoint subscriber)
    {
        OnRemove(_subscriberUpdates, subscriber);
        subscriber.Node.RemoveSubscriber(subscriber);
    }

    private struct PoolingList<T> : IDisposable
    {
        private T[] _data;
        private int _count = 0;

        public PoolingList()
        {
            _data = ArrayPool<T>.Shared.Rent(8);
        }

        public readonly int Count => _count;

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