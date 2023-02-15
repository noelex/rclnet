namespace Rcl.Utils;

static class ObjectPool
{
    public static T Rent<T>()
        where T : class, new()
    {
        return ObjectPool<T>.Shared.Rent();
    }

    public static void Return<T>(T obj)
        where T : class, new()
    {
        ObjectPool<T>.Shared.Return(obj);
    }
}

internal class ObjectPool<T> where T : class, new()
{
    private SpinLock _lock = new();
    private readonly Queue<T> _queue = new();

    public T Rent()
    {
        bool success = false;
        try
        {
            _lock.Enter(ref success);
            var obj = _queue.TryDequeue(out var result) ? result : new();
            return obj;
        }
        finally
        {
            if (success) _lock.Exit();
        }
    }

    public void Return(T obj)
    {
        bool success = false;
        try
        {
            _lock.Enter(ref success);
            _queue.Enqueue(obj);
        }
        finally
        {
            if (success) _lock.Exit();
        }
    }

    public static ObjectPool<T> Shared { get; } = new();
}