using Rcl.Interop;
using System.Runtime.CompilerServices;

namespace Rcl.NET.Tests;

public class OptionsLayoutTests
{
    [Fact]
    public void PublisherRmwOptionsMatchDistributionFamilies()
    {
        Assert.Equal(IntPtr.Size, Unsafe.SizeOf<RclFoxy.rmw_publisher_options_t>());
        Assert.Equal(2 * IntPtr.Size, Unsafe.SizeOf<RclHumble.rmw_publisher_options_t>());
        Assert.Equal(2 * IntPtr.Size, Unsafe.SizeOf<RclIron.rmw_publisher_options_t>());
    }

    [Fact]
    public void SubscriptionRmwOptionsMatchDistributionFamilies()
    {
        var foxySize = Align(IntPtr.Size + sizeof(bool), IntPtr.Size);
        var commonSize = Align(
            Align(IntPtr.Size + sizeof(bool), sizeof(int)) + sizeof(int),
            IntPtr.Size) + IntPtr.Size;

        Assert.Equal(foxySize, Unsafe.SizeOf<RclFoxy.rmw_subscription_options_t>());
        Assert.Equal(commonSize, Unsafe.SizeOf<RclHumble.rmw_subscription_options_t>());
        Assert.Equal(commonSize, Unsafe.SizeOf<RclIron.rmw_subscription_options_t>());
        Assert.Equal(commonSize + IntPtr.Size, Unsafe.SizeOf<RclLyrical.rmw_subscription_options_t>());

        Assert.Equal(commonSize, OffsetOfAcceptableBufferBackends());
    }

    [Fact]
    public void IronAndLaterRclOptionsIncludeDisableLoanedMessage()
    {
        var ironPublisher = default(RclIron.rcl_publisher_options_t);
        var ironSubscription = default(RclIron.rcl_subscription_options_t);
        var lyricalSubscription = default(RclLyrical.rcl_subscription_options_t);

        Assert.Equal(
            OffsetOf(ref ironPublisher, ref ironPublisher.rmw_publisher_options) +
                Unsafe.SizeOf<RclIron.rmw_publisher_options_t>(),
            OffsetOf(ref ironPublisher, ref ironPublisher.disable_loaned_message));
        Assert.Equal(
            OffsetOf(ref ironSubscription, ref ironSubscription.rmw_subscription_options) +
                Unsafe.SizeOf<RclIron.rmw_subscription_options_t>(),
            OffsetOf(ref ironSubscription, ref ironSubscription.disable_loaned_message));
        Assert.Equal(
            OffsetOf(ref lyricalSubscription, ref lyricalSubscription.rmw_subscription_options) +
                Unsafe.SizeOf<RclLyrical.rmw_subscription_options_t>(),
            OffsetOf(ref lyricalSubscription, ref lyricalSubscription.disable_loaned_message));

        Assert.Equal(
            Align(OffsetOf(ref ironPublisher, ref ironPublisher.disable_loaned_message) + sizeof(bool), IntPtr.Size),
            Unsafe.SizeOf<RclIron.rcl_publisher_options_t>());
        Assert.Equal(
            Align(OffsetOf(ref ironSubscription, ref ironSubscription.disable_loaned_message) + sizeof(bool), IntPtr.Size),
            Unsafe.SizeOf<RclIron.rcl_subscription_options_t>());
        Assert.Equal(
            Align(OffsetOf(ref lyricalSubscription, ref lyricalSubscription.disable_loaned_message) + sizeof(bool), IntPtr.Size),
            Unsafe.SizeOf<RclLyrical.rcl_subscription_options_t>());
    }

    [Fact]
    public void RclOptionSizesIncreaseAtAbiBoundaries()
    {
        Assert.Equal(
            Unsafe.SizeOf<RclFoxy.rcl_publisher_options_t>() + IntPtr.Size,
            Unsafe.SizeOf<RclHumble.rcl_publisher_options_t>());
        Assert.Equal(
            Unsafe.SizeOf<RclHumble.rcl_publisher_options_t>() + IntPtr.Size,
            Unsafe.SizeOf<RclIron.rcl_publisher_options_t>());
        Assert.Equal(
            Unsafe.SizeOf<RclIron.rcl_subscription_options_t>() + IntPtr.Size,
            Unsafe.SizeOf<RclLyrical.rcl_subscription_options_t>());
    }

    private static int Align(int value, int alignment)
        => (value + alignment - 1) / alignment * alignment;

    private static int OffsetOf<T, TField>(ref T value, ref TField field)
    {
        ref var start = ref Unsafe.As<T, byte>(ref value);
        ref var target = ref Unsafe.As<TField, byte>(ref field);
        return checked((int)Unsafe.ByteOffset(ref start, ref target));
    }

    private static unsafe int OffsetOfAcceptableBufferBackends()
    {
        var value = default(RclLyrical.rmw_subscription_options_t);
        return checked((int)((byte*)&value.acceptable_buffer_backends - (byte*)&value));
    }
}
