using Rcl.Graph;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class SnapshotCollectionTests(ITestOutputHelper output)
{
    [Fact]
    public void ReadsExcludeUncommittedChangesEvenWithoutACachedSnapshot()
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<object>(publisher);
        var committed = new object();
        var pending = new object();
        collection.Stage(committed, true);
        publisher.Commit();

        // Discovery may fail after staging changes, before any reader creates a snapshot.
        collection.Stage(committed, false);
        collection.Stage(pending, true);
        var snapshot = collection.GetSnapshot();
        Assert.Same(committed, Assert.Single(snapshot));
        Assert.Same(snapshot, collection.GetSnapshot());
        Assert.Throws<NotSupportedException>(() => ((ICollection<object>)snapshot).Clear());

        publisher.Commit();
        Assert.Same(pending, Assert.Single(collection.GetSnapshot()));
        Assert.Same(committed, Assert.Single(snapshot));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void EqualReplacementPublishesTheNewReference(bool separateCommits)
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<Item>(publisher);
        var original = new Item(1, 0);
        var replacement = new Item(1, 0);
        collection.Stage(original, true);
        publisher.Commit();
        var snapshot = collection.GetSnapshot();

        collection.Stage(original, false);
        if (separateCommits)
        {
            publisher.Commit();
        }

        collection.Stage(replacement, true);
        publisher.Commit();

        Assert.Equal(original, replacement);
        Assert.Same(replacement, Assert.Single(collection.GetSnapshot()));
        Assert.Same(original, Assert.Single(snapshot));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ChangesAcrossCommitsRestoreTheCachedSnapshot(bool initiallyPresent)
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<object>(publisher);
        var member = new object();
        if (initiallyPresent)
        {
            collection.Stage(member, true);
            publisher.Commit();
        }

        var snapshot = collection.GetSnapshot();
        var meter = new AllocationMeter(output);
        meter.Measure("snapshot-coalesced-commits", 1000, () =>
        {
            collection.Stage(member, !initiallyPresent);
            publisher.Commit();
            collection.Stage(member, initiallyPresent);
            publisher.Commit();
            collection.GetSnapshot();
        }, zeroAllocation: true);

        Assert.Same(snapshot, collection.GetSnapshot());
        Assert.Equal(initiallyPresent ? 1 : 0, snapshot.Count);
    }

    [Fact]
    public void MaterializationResetsTheCommittedChangeBaseline()
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<object>(publisher);
        var first = new object();
        var second = new object();
        collection.Stage(first, true);
        publisher.Commit();
        var original = collection.GetSnapshot();

        collection.Stage(second, true);
        publisher.Commit();
        var expanded = collection.GetSnapshot();
        Assert.Equal(2, expanded.Count);

        collection.Stage(first, false);
        publisher.Commit();
        collection.Stage(first, true);
        publisher.Commit();
        Assert.Same(expanded, collection.GetSnapshot());

        collection.Stage(second, false);
        publisher.Commit();
        var restored = collection.GetSnapshot();
        Assert.NotSame(expanded, restored);
        Assert.Same(first, Assert.Single(restored));
        Assert.Single(original);
        Assert.Equal(2, expanded.Count);
    }

    [Fact]
    public void CancelledChangesKeepTheCachedSnapshot()
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<object>(publisher);
        var member = new object();
        collection.Stage(member, true);
        publisher.Commit();
        var snapshot = collection.GetSnapshot();
        var transient = new object();

        collection.Stage(member, false);
        collection.Stage(transient, true);
        collection.Stage(member, true);
        collection.Stage(transient, false);
        publisher.Commit();

        Assert.Same(snapshot, collection.GetSnapshot());
        Assert.Same(member, Assert.Single(snapshot));
    }

    [Fact]
    public void RepeatedCommitsWithoutReadsDoNotAllocateSnapshots()
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<object>(publisher);
        for (var i = 0; i < 1024; i++)
        {
            collection.Stage(new object(), true);
        }

        publisher.Commit();
        var member = new object();
        var meter = new AllocationMeter(output);
        meter.Measure("snapshot-update-without-read", 1000, () =>
        {
            collection.Stage(member, true);
            publisher.Commit();
            collection.Stage(member, false);
            publisher.Commit();
        }, zeroAllocation: true);

        var snapshot = collection.GetSnapshot();
        Assert.Equal(1024, snapshot.Count);
        meter.Measure("snapshot-cached-read", 1000, () => collection.GetSnapshot(), zeroAllocation: true);
        Assert.Same(snapshot, collection.GetSnapshot());
        meter.Measure("snapshot-update-and-read", 1000, () =>
        {
            collection.Stage(member, true);
            publisher.Commit();
            collection.GetSnapshot();
            collection.Stage(member, false);
            publisher.Commit();
            collection.GetSnapshot();
        });
    }

    [Fact]
    public async Task ConcurrentReadsSeeCompleteBatches()
    {
        var publisher = new SnapshotPublisher();
        var collection = new SnapshotCollection<Item>(publisher);
        var members = Enumerable.Range(0, 32).Select(i => new Item(0, i)).ToArray();
        foreach (var item in members)
        {
            collection.Stage(item, true);
        }

        publisher.Commit();
        var start = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var writer = Task.Run(async () =>
        {
            await start.Task;
            for (var generation = 1; generation <= 1000; generation++)
            {
                foreach (var item in members)
                {
                    collection.Stage(item, false);
                }

                members = Enumerable.Range(0, 32).Select(i => new Item(generation, i)).ToArray();
                foreach (var item in members)
                {
                    collection.Stage(item, true);
                }

                publisher.Commit();
            }
        });
        var reader = Task.Run(async () =>
        {
            await start.Task;
            for (var i = 0; i < 2000; i++)
            {
                var snapshot = collection.GetSnapshot();
                Assert.Equal(32, snapshot.Count);
                Assert.Single(snapshot.Select(item => item.Generation).Distinct());
                Assert.Equal(32, snapshot.Select(item => item.Index).Distinct().Count());
            }
        });

        start.SetResult();
        await Task.WhenAll(writer, reader);
        Assert.All(collection.GetSnapshot(), item => Assert.Equal(1000, item.Generation));
    }

    private sealed record Item(int Generation, int Index);
}
