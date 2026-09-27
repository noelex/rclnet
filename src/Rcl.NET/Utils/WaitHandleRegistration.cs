using Rcl.SafeHandles;

namespace Rcl.Utils;

internal readonly struct WaitHandleRegistration : IDisposable
{
    internal readonly WaitSetRegistration? Entry;

    internal WaitHandleRegistration(WaitSetRegistration entry) => Entry = entry;

    public static readonly WaitHandleRegistration Empty = new();

    public bool IsEmpty => Entry is null;

    public void Dispose() => Entry?.Context.UnregisterWaitHandle(Entry);
}

internal sealed class WaitSetRegistration
{
    internal readonly RclContext Context;
    internal readonly long Token;
    internal readonly RclObjectHandle WaitHandle;
    internal readonly Action<RclObjectHandle, object?> Callback;
    internal readonly object? State;
    internal readonly Action<object?>? Closed;
    internal readonly Action<object?>? OnDetached;
    internal bool RemoveRequested, Detached;
    internal List<(Action<object?> Callback, object? State)>? Cleanup;

    internal WaitSetRegistration(RclContext context, long token, RclObjectHandle handle,
        Action<RclObjectHandle, object?> callback, object? state, Action<object?>? closed, Action<object?>? detached)
    {
        Context = context;
        Token = token;
        WaitHandle = handle;
        Callback = callback;
        State = state;
        Closed = closed;
        OnDetached = detached;
    }
}
