using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class ThreadSafetyTests
{
    [Fact]
    public async Task ConcurrentContextCreation()
    {
        var tasks = Enumerable.Range(0, Environment.ProcessorCount).Select(x => Task.Run(() => new RclContext(TestConfig.DefaultContextArguments))).ToArray();
        var contexts = await Task.WhenAll(tasks);
        await Task.WhenAll(contexts.Select(async x => await x.DisposeAsync()));
        Assert.True(true);
    }

    [Theory]
    [InlineData(RclClockType.Steady)]
    [InlineData(RclClockType.System)]
    [InlineData(RclClockType.Ros)]
    public async Task ConcurrentTimerCreation(RclClockType clockType)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var clock = clockType switch
        {
            RclClockType.Steady => RclClock.SteadyClock,
            RclClockType.System => RclClock.SystemClock,
            RclClockType.Ros => node.Clock,
            _ => throw new NotSupportedException()
        };

        var tasks = Enumerable.Range(0, Environment.ProcessorCount).Select(x => Task.Run(() => context.CreateTimer(clock, TimeSpan.FromMilliseconds(200))));
        var timers = await Task.WhenAll(tasks);
        using var cts = new CancellationTokenSource(1000);
        await Task.WhenAll(timers.Select(async x => await x.WaitOneAsync(cts.Token)));
        await Task.WhenAll(timers.Select(x => Task.Run(() => x.Dispose())));
    }

    [Theory]
    [InlineData(RclClockType.Steady)]
    [InlineData(RclClockType.System)]
    [InlineData(RclClockType.Ros)]
    public async Task ConcurrentTimerCreationAndDisposal(RclClockType clockType)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var clock = clockType switch
        {
            RclClockType.Steady => RclClock.SteadyClock,
            RclClockType.System => RclClock.SystemClock,
            RclClockType.Ros => node.Clock,
            _ => throw new NotSupportedException()
        };

        var timers = await Task.WhenAll(
            Enumerable.Range(0, Environment.ProcessorCount)
            .Select(x => Task.Run(() => context.CreateTimer(clock, TimeSpan.FromMilliseconds(200))))
        );
        await Task.WhenAll(timers.Select(x => Task.Run(() => x.Dispose())));

        var tasks2 = Enumerable.Range(0, Environment.ProcessorCount)
            .Select(x => Task.Run(() => context.CreateTimer(clock, TimeSpan.FromMilliseconds(200)))).ToArray();
        await Task.WhenAll(timers.Select(x => Task.Run(() => x.Dispose())).Merge(tasks2));

        using var cts = new CancellationTokenSource(1000);
        await Task.WhenAll(tasks2.Select(async x => await x.Result.WaitOneAsync(cts.Token)));
        await Task.WhenAll(tasks2.Select(x => Task.Run(() => x.Result.Dispose())));
    }

    [Fact]
    public async Task ConcurrentGuardConditionCreation()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        var tasks = Enumerable.Range(0, Environment.ProcessorCount).Select(x => Task.Run(() => context.CreateGuardCondition()));
        var objects = await Task.WhenAll(tasks);
        await Task.WhenAll(objects.Select(x => Task.Run(() => x.Dispose())));
    }
}

static class LinqExtensions
{
    public static IEnumerable<T> Merge<T>(this IEnumerable<T> src, IEnumerable<T> other)
    {
        using var sourceEnumerator = src.GetEnumerator();
        using var otherEnumerator = other.GetEnumerator();

        bool continueSource = true, continueOther = true;
        do
        {
            if (continueSource)
            {
                if (sourceEnumerator.MoveNext())
                {
                    yield return sourceEnumerator.Current;
                }
                else
                {
                    continueSource = false;
                }
            }

            if (continueOther)
            {
                if (otherEnumerator.MoveNext())
                {
                    yield return otherEnumerator.Current;
                }
                else
                {
                    continueOther = false;
                }
            }

        } while (continueSource || continueOther);
    }
}
