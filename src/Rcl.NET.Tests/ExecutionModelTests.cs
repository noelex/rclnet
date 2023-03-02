using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.NET.Tests;
public class ExecutionModelTests
{
    [Fact]
    public async Task WithSynchronizationContextEnabled()
    {
        var context = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: true);
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);

        await Task.Yield();
        Assert.True(context.IsCurrent);

        await Task.Delay(1);
        Assert.True(context.IsCurrent);

        await Task.Run(() =>
        {
            Assert.False(context.IsCurrent);
        });
        Assert.True(context.IsCurrent);

        await Task.Delay(1).ConfigureAwait(false);
        Assert.False(context.IsCurrent);

        await Task.Yield();
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);
    }

    [Fact]
    public async Task WithSynchronizationContextDisabled()
    {
        var context = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: false);
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);

        await Task.Yield();
        Assert.False(context.IsCurrent);

        await Task.Delay(1);
        Assert.False(context.IsCurrent);

        await Task.Run(() =>
        {
            Assert.False(context.IsCurrent);
        });
        Assert.False(context.IsCurrent);

        await Task.Delay(1).ConfigureAwait(false);
        Assert.False(context.IsCurrent);

        await Task.Yield();
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);
    }
}
