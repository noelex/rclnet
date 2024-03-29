﻿namespace Rcl.Utils;

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

internal class ObjectPool<T> : IDisposable where T : class, new()
{
    private SpinLock _lock = new();
    private readonly Queue<T> _queue = new();

    public T Rent()
    {
        using (ScopedLock.Lock(ref _lock))
        {
            var obj = _queue.TryDequeue(out var result) ? result : new();
            return obj;
        }
    }

    public void Return(T obj)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            _queue.Enqueue(obj);
        }
    }

    public void Dispose()
    {
        using (ScopedLock.Lock(ref _lock))
        {
            while (_queue.TryDequeue(out var obj))
            {
                if (obj is IDisposable d) d.Dispose();
            }
        }
    }

    public static ObjectPool<T> Shared { get; } = new();
}