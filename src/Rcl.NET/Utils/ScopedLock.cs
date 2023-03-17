namespace Rcl.Utils;

ref struct ScopedLock
{
    private readonly bool _taken;
    private ref SpinLock _spinLock;

    public ScopedLock(ref SpinLock spinLock)
    {
        _spinLock = ref spinLock;
        _spinLock.Enter(ref _taken);
    }

    public static ScopedLock Lock(ref SpinLock spinLock)
    {
        return new ScopedLock(ref spinLock);
    }

    public void Dispose()
    {
        if (_taken)
        {
            _spinLock.Exit();
        }
    }
}