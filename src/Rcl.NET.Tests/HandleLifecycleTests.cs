using System.Collections.Concurrent;

namespace Rcl.NET.Tests;

/// <summary>Tests the SafeHandle behavior on which lifecycle leases will rely.</summary>
public class HandleLifecycleTests
{
    /// <summary>A held reference delays release despite repeated disposal.</summary>
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

    /// <summary>IsClosed is published before native cleanup completes.</summary>
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

    /// <summary>A physically released handle rejects another reference.</summary>
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
