using Rcl.Actions;
using Rcl.Actions.Client;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Tf2;
using System.Text;

namespace Rcl.NET.Tests;

public class ObserverSnapshotTests
{
    [Fact]
    public async Task TypedSubscriptionObserverSnapshots()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        using var subscription = node.CreateSubscription<Time>(publisher.Name);
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(30));
        await using var reader = subscription.ReadAllAsync(cancellation.Token).GetAsyncEnumerator();

        while (publisher.Subscribers == 0)
        {
            await Task.Delay(10, cancellation.Token);
        }

        await VerifySnapshotsAsync(subscription, async () =>
        {
            var received = reader.MoveNextAsync();
            publisher.Publish(new Time());
            Assert.True(await received);
            // The channel write precedes observer dispatch; wait until that callback has returned.
            await context.Yield();
        }, async () =>
        {
            subscription.Dispose();
            await context.Yield();
        });
    }

    [Fact]
    public async Task ActionFeedbackObserverSnapshots()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(NameGenerator.GenerateActionName());
        var goal = new ActionGoalContext<LookupTransformActionResult, LookupTransformActionFeedback>(
            Guid.NewGuid(), (IActionClientImpl)client, Encoding.UTF8);

        Assert.False(goal.HasFeedbackListeners);
        await VerifySnapshotsAsync(goal, () =>
        {
            goal.OnFeedbackReceived(RosMessageBuffer.Create<LookupTransformActionFeedback>());
            return ValueTask.CompletedTask;
        }, () =>
        {
            goal.OnStatusChanged(ActionGoalStatus.Succeeded);
            return ValueTask.CompletedTask;
        });
        Assert.False(goal.HasFeedbackListeners);
    }

    private static async Task VerifySnapshotsAsync<T>(IObservable<T> source,
        Func<ValueTask> dispatch, Func<ValueTask> complete)
    {
        var observer = new Observer<T>();
        using var first = source.Subscribe(observer);
        using var duplicate = source.Subscribe(observer);
        await dispatch();
        Assert.Equal(2, observer.NextCount);
        first.Dispose();
        first.Dispose();
        await dispatch();
        Assert.Equal(3, observer.NextCount);
        duplicate.Dispose();

        // Concurrent changes must not overwrite one another's snapshot.
        var registrations = new IDisposable[32];
        Parallel.For(0, registrations.Length, i => registrations[i] = source.Subscribe(observer));
        await dispatch();
        Assert.Equal(35, observer.NextCount);
        Parallel.For(0, registrations.Length / 2, i => registrations[i * 2].Dispose());
        await dispatch();
        Assert.Equal(51, observer.NextCount);

        foreach (var registration in registrations)
        {
            registration.Dispose();
        }

        await dispatch();
        Assert.Equal(51, observer.NextCount);

        var lateObserver = new Observer<T>();
        IDisposable? self = null;
        IDisposable? late = null;
        self = source.Subscribe(new Observer<T>(() =>
        {
            // A callback can change subscriptions from another thread without waiting on a dispatch lock.
            Task.Run(() =>
            {
                self!.Dispose();
                late = source.Subscribe(lateObserver);
            }).WaitAsync(TimeSpan.FromSeconds(5)).GetAwaiter().GetResult();
        }));

        using (self)
        {
            await dispatch();
            Assert.Equal(0, lateObserver.NextCount);
            await dispatch();
            Assert.Equal(1, lateObserver.NextCount);
        }

        using (late)
        {
            await complete();
            await complete();
            Assert.Equal(1, lateObserver.CompletedCount);
            Assert.Equal(0, observer.CompletedCount);
        }
    }

    private sealed class Observer<T>(Action? next = null) : IObserver<T>
    {
        public int NextCount;
        public int CompletedCount;

        public void OnNext(T value)
        {
            Interlocked.Increment(ref NextCount);
            next?.Invoke();
        }

        public void OnCompleted()
        {
            Interlocked.Increment(ref CompletedCount);
        }

        public void OnError(Exception error)
        {
            throw error;
        }
    }
}
