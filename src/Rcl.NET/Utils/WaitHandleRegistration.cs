namespace Rcl.Utils;

internal readonly struct WaitHandleRegistration : IDisposable
{
    private readonly long _token;
    private readonly RclContext _context;
    private readonly Action<RclContext, long> _unregisterCallback;

    public WaitHandleRegistration(RclContext context, Action<RclContext, long> unregisterCallback, long token)
    {
        _unregisterCallback = unregisterCallback;
        _token = token;
        _context = context;
    }

    public static readonly WaitHandleRegistration Empty = new();

    public bool IsEmpty => _unregisterCallback == null;

    public void Dispose()
    {
        _unregisterCallback(_context, _token);
    }
}
