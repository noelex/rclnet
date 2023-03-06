using System.Threading.Channels;

namespace Rcl.Graph;

public partial class RosGraph
{
    private readonly Channel<RosGraphEvent> _channel = Channel.CreateBounded<RosGraphEvent>(new BoundedChannelOptions(1)
    {
        SingleWriter = true,
        SingleReader = false,
        AllowSynchronousContinuations = false,
        FullMode = BoundedChannelFullMode.DropOldest
    });

    /// <summary>
    /// Read all graph change events asynchronously.
    /// </summary>
    public  IAsyncEnumerable<RosGraphEvent> ReadEventsAsync(CancellationToken cancellationToken = default)
        => _channel.Reader.ReadAllAsync(cancellationToken);

    /// <summary>
    /// Subscribe to current <see cref="RosGraph"/> object to receive graph change events.
    /// </summary>
    /// <returns>
    /// A <see cref="IDisposable"/> object which will unsubscribe from current
    /// <see cref="RosGraph"/> object when disposed.
    /// </returns>
    public IDisposable Subscribe(IObserver<RosGraphEvent> observer)
    {
        var id = Interlocked.Increment(ref _subscriberId);
        _observers[id] = observer;

        return new RosGraphEventSubscription(this, id);
    }

    private void Unsubscribe(long id)
    {
        _observers.Remove(id, out _);
    }

    internal void Complete()
    {
        _channel.Writer.TryComplete();
        foreach(var obs in _observers.Values)
        {
            obs.OnCompleted();
        }
    }

    private class RosGraphEventSubscription : IDisposable
    {
        private readonly RosGraph _graph;
        private readonly long _id;

        public RosGraphEventSubscription(RosGraph graph, long id)
        {
            _graph = graph;
            _id = id;
        }

        public void Dispose()
        {
            _graph.Unsubscribe(_id);
        }
    }
}