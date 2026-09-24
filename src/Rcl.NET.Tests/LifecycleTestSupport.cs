using Rcl.SafeHandles;
using System.Collections.Concurrent;

namespace Rcl.NET.Tests;

internal sealed class LifecycleCheckpoint : IDisposable
{
    private readonly ManualResetEventSlim _resume = new();
    private readonly TaskCompletionSource _entered = new(TaskCreationOptions.RunContinuationsAsynchronously);

    public Task Entered => _entered.Task;

    public bool TimedOut { get; private set; }

    public void Pause()
    {
        _entered.TrySetResult();
        // A timeout prevents a failed test from leaving the event loop blocked forever.
        TimedOut = !_resume.Wait(TimeSpan.FromSeconds(30));
    }

    public void Resume() => _resume.Set();

    public void Dispose() => _resume.Dispose();
}

internal sealed unsafe class FakeRclHandle : RclObjectHandle<int>
{
    private readonly ConcurrentQueue<string> _releases;
    private readonly string _name;
    private readonly LifecycleCheckpoint? _checkpoint;

    public FakeRclHandle(ConcurrentQueue<string> releases, string name = "handle",
        LifecycleCheckpoint? checkpoint = null)
    {
        _releases = releases;
        _name = name;
        _checkpoint = checkpoint;
        MarkInitialized();
    }

    protected override bool ReleaseHandleCore(int* ptr)
    {
        _releases.Enqueue($"{_name}:enter");
        _checkpoint?.Pause();
        _releases.Enqueue($"{_name}:exit");
        return true;
    }
}
