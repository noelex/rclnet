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