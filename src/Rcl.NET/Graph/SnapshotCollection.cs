namespace Rcl.Graph;

internal sealed class SnapshotPublisher
{
    // Only the event loop stages and commits changes; readers synchronize with commits.
    private readonly HashSet<ISnapshotCollection> _pending = new();

    internal object Gate { get; } = new();

    internal void Register(ISnapshotCollection collection)
    {
        _pending.Add(collection);
    }

    internal void Commit()
    {
        lock (Gate)
        {
            foreach (var collection in _pending)
            {
                collection.Commit();
            }

            _pending.Clear();
        }
    }
}

internal interface ISnapshotCollection
{
    void Commit();
}

internal sealed class SnapshotCollection<T>(SnapshotPublisher publisher) : ISnapshotCollection where T : class
{
    private HashSet<T>? _members;
    private Dictionary<T, bool>? _pending;
    private IReadOnlyCollection<T>? _snapshot = Array.Empty<T>();

    internal void Stage(T item, bool adding)
    {
        // Value-equal replacement objects must remain distinct from the old references.
        _pending ??= new(ReferenceEqualityComparer.Instance);
        if (_pending.TryGetValue(item, out var previous) && previous != adding)
        {
            _pending.Remove(item);
        }
        else
        {
            _pending[item] = adding;
        }

        publisher.Register(this);
    }

    void ISnapshotCollection.Commit()
    {
        var changed = false;
        foreach (var (item, adding) in _pending!)
        {
            if (adding)
            {
                _members ??= new(ReferenceEqualityComparer.Instance);
                changed |= _members.Add(item);
            }
            else
            {
                changed |= _members!.Remove(item);
            }
        }

        if (changed)
        {
            Volatile.Write(ref _snapshot, null);
        }

        _pending.Clear();
    }

    internal IReadOnlyCollection<T> GetSnapshot()
    {
        var snapshot = Volatile.Read(ref _snapshot);
        if (snapshot != null)
        {
            return snapshot;
        }

        lock (publisher.Gate)
        {
            // Copy only committed membership, including when discovery has since failed.
            snapshot = _snapshot;
            if (snapshot == null)
            {
                snapshot = _members!.Count == 0
                    ? Array.Empty<T>()
                    : Array.AsReadOnly(_members.ToArray());
                Volatile.Write(ref _snapshot, snapshot);
            }

            return snapshot;
        }
    }
}
