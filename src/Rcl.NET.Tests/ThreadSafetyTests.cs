using System.Xml.Linq;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class ThreadSafetyTests
{
    private static readonly int s_concurrency = Environment.ProcessorCount;

    [SkippableTheory]
    [InlineData(RclClockType.Steady)]
    [InlineData(RclClockType.System)]
    [InlineData(RclClockType.Ros)]
    public async Task ConcurrentTimerCreationAndDisposal_MultipleContexts(RclClockType clockType)
    {
        Skip.If(TestConfig.GitHubActions, "Tests for concurrent context creation and disposable are disabled in CI.");

        await Task.WhenAll(
            Enumerable.Range(0, s_concurrency)
            .Select(x => Random.Shared.Next(1, 5))
            .Select(CreateContextAndTestTimerAsync)
        );

        async Task CreateContextAndTestTimerAsync(int timeout)
        {
            await using var context = new RclContext(TestConfig.DefaultContextArguments);
            using var node = context.CreateNode(NameGenerator.GenerateNodeName());

            await CreateTimerWaitAndDisposeAsync(node, clockType, timeout);
        }
    }

    [Theory]
    [InlineData(RclClockType.Steady)]
    [InlineData(RclClockType.System)]
    [InlineData(RclClockType.Ros)]
    public async Task ConcurrentTimerCreationAndDisposal_SingleContext(RclClockType clockType)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        await Task.WhenAll(
            Enumerable.Range(0, s_concurrency)
            .Select(x => Random.Shared.Next(1, 5))
            .Select(x => CreateTimerWaitAndDisposeAsync(node, clockType, x))
        );
    }

    [Fact]
    public async Task ConcurrentGuardConditionCreation_SingleContext()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        await Task.WhenAll(
            Enumerable.Range(0, s_concurrency)
            .Select(x => Random.Shared.Next(1, 5))
            .Select(x => CreateGuardConditionWaitAndDisposeAsync(context, x))
        );
    }

    private static async Task CreateTimerWaitAndDisposeAsync(IRclNode node, RclClockType type, int timeout)
    {
        var clock = type switch
        {
            RclClockType.Steady => RclClock.SteadyClock,
            RclClockType.System => RclClock.SystemClock,
            RclClockType.Ros => node.Clock,
            _ => throw new NotSupportedException()
        };

        using var cts = new CancellationTokenSource(1000);
        using var timer = node.Context.CreateTimer(clock, TimeSpan.FromMilliseconds(timeout));
        await timer.WaitOneAsync(cts.Token);
    }

    private static async Task CreateGuardConditionWaitAndDisposeAsync(IRclContext context, int timeout)
    {
        using var cts = new CancellationTokenSource(1000);
        using var gc = context.CreateGuardCondition();
        _ = Task.Delay(timeout).ContinueWith(x => gc.Trigger());
        await gc.WaitOneAsync(cts.Token);
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
