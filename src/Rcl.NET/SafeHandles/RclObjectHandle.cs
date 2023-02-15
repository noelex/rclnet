using Microsoft.Win32.SafeHandles;
using System.Runtime.InteropServices;

namespace Rcl.SafeHandles;

public abstract class RclObjectHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    protected RclObjectHandle(bool ownsHandle) : base(ownsHandle)
    {
    }
}

internal unsafe abstract class RclObjectHandle<T> : RclObjectHandle
    where T : unmanaged
{
    protected RclObjectHandle() : base(true)
    {
        SetHandle(Marshal.AllocHGlobal(sizeof(T)));
    }

    protected RclObjectHandle(IntPtr handle) : base(false)
    {
        SetHandle(handle);
    }

    public void ThrowIfInvalidOrClosed()
    {
        if (IsInvalid || IsClosed)
        {
            throw new ObjectDisposedException(GetType().Name);
        }
    }

    public T* Object
    {
        get
        {
            ThrowIfInvalidOrClosed();
            return (T*)handle.ToPointer();
        }
    }

    protected abstract void ReleaseHandleCore(T* ptr);

    protected override bool ReleaseHandle()
    {
        ReleaseHandleCore((T*)handle);
        Marshal.FreeHGlobal(handle);
        return true;
    }
}