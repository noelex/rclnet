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
        if (TryBeginClose())
        {
            DisposeCore();
        }
    }

    protected virtual bool TryBeginClose() => Handle.TryBeginClose();

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

    protected override bool TryBeginClose()
    {
        lock (Context.RegistrationGate)
        {
            return Handle.TryBeginClose();
        }
    }

    protected override void DisposeCore()
    {
        Context.ScheduleCleanup(state =>
        {
            var self = (RclContextualObject<T>)state!;
            self.Handle.Dispose();
        }, this);
    }
}
