using System.Runtime.CompilerServices;

namespace Rcl.Graph;

public partial class RosGraph
{
    /// <summary>
    /// Read all graph change events asynchronously.
    /// </summary>
    public IAsyncEnumerable<RosGraphEvent> ReadEventsAsync(CancellationToken cancellationToken = default)
    {
        if (!cancellationToken.CanBeCanceled)
        {
            return this.ToAsyncEnumerable();
        }

        return Impl(cancellationToken);

        async IAsyncEnumerable<RosGraphEvent> Impl([EnumeratorCancellation] CancellationToken cancellationToken)
        {
            await foreach (var e in this.ToAsyncEnumerable().WithCancellation(cancellationToken))
            {
                yield return e;
            }
        }
    }

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
        foreach (var obs in _observers.Values)
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