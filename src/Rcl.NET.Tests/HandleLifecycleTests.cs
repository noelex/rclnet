using System.Collections.Concurrent;

namespace Rcl.NET.Tests;

public class HandleLifecycleTests
{
    [Fact]
    public void ReferenceDelaysReleaseUntilReturned()
    {
        var releases = new ConcurrentQueue<string>();
        using var handle = new FakeRclHandle(releases);
        bool added = false;
        try
        {
            handle.DangerousAddRef(ref added);
            handle.Dispose();
            handle.Dispose();
            Assert.False(handle.IsClosed);
            Assert.Empty(releases);
        }
        finally
        {
            if (added) handle.DangerousRelease();
        }

        Assert.True(handle.IsClosed);
        Assert.Equal(new[] { "handle:enter", "handle:exit" }, releases.ToArray());
    }

    [Fact]
    public async Task ClosedDoesNotMeanReleaseCompleted()
    {
        var releases = new ConcurrentQueue<string>();
        using var checkpoint = new LifecycleCheckpoint();
        using var handle = new FakeRclHandle(releases, checkpoint: checkpoint);
        var disposing = Task.Run(handle.Dispose);
        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.True(handle.IsClosed);
            Assert.False(disposing.IsCompleted);
            Assert.Equal(new[] { "handle:enter" }, releases.ToArray());
        }
        finally
        {
            checkpoint.Resume();
            await disposing.WaitAsync(TimeSpan.FromSeconds(10));
        }

        Assert.False(checkpoint.TimedOut);
        Assert.Equal(new[] { "handle:enter", "handle:exit" }, releases.ToArray());
    }

    [Fact]
    public void ReleasedHandleRejectsAddRef()
    {
        using var handle = new FakeRclHandle(new());
        handle.Dispose();
        bool added = false;
        Assert.Throws<ObjectDisposedException>(() => handle.DangerousAddRef(ref added));
        Assert.False(added);
    }
}
