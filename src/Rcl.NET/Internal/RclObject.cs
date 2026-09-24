using Rcl.SafeHandles;

namespace Rcl.Internal;

internal abstract class RclObject<T> : IDisposable where T : RclObjectHandle
{
    public RclObject(T handle)
    {
        Handle = handle;
    }

    internal T Handle { get; }

    public void Dispose()
    {
        if (Handle.TryBeginClose()) DisposeCore();
    }

    protected virtual void DisposeCore()
    {
        Handle.Dispose();
    }
}

internal abstract class RclContextualObject<T> : RclObject<T> where T : RclObjectHandle
{
    public RclContextualObject(RclContext context, T handle)
        : base(handle)
    {
        Context = context;
    }

    public RclContext Context { get; }

    protected override void DisposeCore()
    {
        Context.SynchronizationContext.Post(state =>
        {
            var self = (RclContextualObject<T>)state!;
            self.Handle.Dispose();
        }, this);
    }
}

