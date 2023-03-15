using Rosidl.Messages.Rcl;
using System.Diagnostics;
using System.Threading;
using Xunit.Abstractions;
using Xunit.Sdk;

namespace Rcl.NET.Tests;

public class ServiceTests
{
    private const int RequestTimeout = 10_000, ServerOnlineTimeout = 1000;

    private readonly ITestOutputHelper ms;

    public ServiceTests(ITestOutputHelper ms)
    {
        this.ms = ms;
    }

    [Fact]
    public async Task ClientRequestTimeout()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());

        await Assert.ThrowsAsync<TimeoutException>(() =>
            client.InvokeAsync(new ListParametersServiceRequest(), 100));
    }

    [Fact]
    public async Task NormalServiceCall()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var response = new ListParametersServiceResponse(new(new[] { "n1", "n2" }, new[] { "p1", "p2" }));
        var service = NameGenerator.GenerateServiceName();

        using var server = node.CreateService<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service, (request, state) =>
            {
                Assert.True(context.IsCurrent);
                return response;
            });

        using var clientNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = clientNode.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service);

        await client.TryWaitForServerAsync(ServerOnlineTimeout);
        var actualResponse = await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);

        Assert.True(response.Result.Names.SequenceEqual(actualResponse.Result.Names));
        Assert.True(response.Result.Prefixes.SequenceEqual(actualResponse.Result.Prefixes));
    }

    [Fact]
    public Task ContinuationOfInvokeAsyncShouldNotExecuteOnEventLoopWhenContextIsCaptured()
    {
        return Task.Run(async () =>
        {
            await using var anotherContext = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: true);

            await using var context = new RclContext(TestConfig.DefaultContextArguments);
            using var node = context.CreateNode(NameGenerator.GenerateNodeName());

            var response = new ListParametersServiceResponse();
            var service = NameGenerator.GenerateServiceName();

            using var server = node.CreateService<
                ListParametersService,
                ListParametersServiceRequest,
                ListParametersServiceResponse>(service, (request, state) =>
                {
                    return response;
                });

            using var clientNode = context.CreateNode(NameGenerator.GenerateNodeName());
            using var client = clientNode.CreateClient<
                ListParametersService,
                ListParametersServiceRequest,
                ListParametersServiceResponse>(service);

            ms.WriteLine($"[{Environment.TickCount64}] Waiting for server...");
            var result = await client.TryWaitForServerAsync(ServerOnlineTimeout);
            ms.WriteLine($"[{Environment.TickCount64}] OK. Online = " +result); 
            await anotherContext.Yield();
        // Now we are on the event loop of anotherContext.

        // Captured the sync context of the anotherContext, we should be on the event loop of anotherContext.
        ms.WriteLine($"[{Environment.TickCount64}] Invoke service on anotherContext...");
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
        ms.WriteLine($"[{Environment.TickCount64}] OK.");
            Assert.True(anotherContext.IsCurrent);

        // Suppressing the sync context.
        ms.WriteLine($"[{Environment.TickCount64}] Invoke service on anotherContext with ConfigureAwait(false)...");
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout).ConfigureAwait(false);
        ms.WriteLine($"[{Environment.TickCount64}] OK.");

        // No captured sync context, we should be on thread pool thread.
        ms.WriteLine($"[{Environment.TickCount64}] Invoke service without any sync context...");
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
        ms.WriteLine($"[{Environment.TickCount64}] OK.");
            Assert.False(context.IsCurrent);

        ms.WriteLine($"[{Environment.TickCount64}] Invoke service without any sync context...");
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout).ConfigureAwait(false);
        ms.WriteLine($"[{Environment.TickCount64}] OK.");
            Assert.False(context.IsCurrent);
        });
    }

    [Fact]
    public async Task ConcurrentServiceCallHandlerShouldStartOnEventLoop()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var response = new ListParametersServiceResponse(new(new[] { "n1", "n2" }, new[] { "p1", "p2" }));
        var service = NameGenerator.GenerateServiceName();

        using var server = node.CreateConcurrentService<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service, (request, state, ct) =>
            {
                Assert.True(context.IsCurrent);
                return Task.FromResult(response);
            }, null);

        using var clientNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = clientNode.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service);

        await client.TryWaitForServerAsync(ServerOnlineTimeout);
        var actualResponse = await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
    }

    [Fact]
    public async Task InvokeWithZeroTimeout()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());

        await Assert.ThrowsAsync<TimeoutException>(() =>
            client.InvokeAsync(new ListParametersServiceRequest(), 0));
    }
}
