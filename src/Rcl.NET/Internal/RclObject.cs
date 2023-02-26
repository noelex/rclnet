using Rcl.SafeHandles;

namespace Rcl.Internal;

internal abstract class RclObject<T> : IDisposable where T : RclObjectHandle
{
    public RclObject(T handle)
    {
        Handle = handle;
    }

    internal T Handle { get; }

    public virtual void Dispose()
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


    public override void Dispose()
    {
        if (!Context.IsCurrent)
        {
            Context.SynchronizationContext.Send(state => ((RclObjectHandle)state!).Dispose(), Handle);
        }
        else
        {
            Handle.Dispose();
        }
    }
}

