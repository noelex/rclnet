using Rosidl.Messages.Geometry;
using System.Text;

namespace Rcl.NET.Tests;

public class MessageTests
{
    [Fact]
    public void MessageRoundTrip()
    {
        var msg = new TwistWithCovarianceStamped(
            header: new(stamp: new(123, 456), frameId: "map"),
            twist: new(
                twist: new(linear: new(1, 2, 3), angular: new(4, 5, 6)),
                covariance: Enumerable.Range(0, 36).Select(x => Random.Shared.NextDouble()).ToArray()));

        var native = new TwistWithCovarianceStamped.Priv();
        try
        {
            msg.WriteTo(ref native, Encoding.UTF8);

            var deserializedMsg = new TwistWithCovarianceStamped(in native, Encoding.UTF8);

            Assert.Equal(msg.Header.Stamp.Sec, deserializedMsg.Header.Stamp.Sec);
            Assert.Equal(msg.Header.Stamp.Nanosec, deserializedMsg.Header.Stamp.Nanosec);
            Assert.Equal(msg.Header.FrameId, deserializedMsg.Header.FrameId);

            Assert.Equal(msg.Twist.Twist.Linear.X, deserializedMsg.Twist.Twist.Linear.X);
            Assert.Equal(msg.Twist.Twist.Linear.Y, deserializedMsg.Twist.Twist.Linear.Y);
            Assert.Equal(msg.Twist.Twist.Linear.Z, deserializedMsg.Twist.Twist.Linear.Z);
            Assert.Equal(msg.Twist.Twist.Angular.X, deserializedMsg.Twist.Twist.Angular.X);
            Assert.Equal(msg.Twist.Twist.Angular.Y, deserializedMsg.Twist.Twist.Angular.Y);
            Assert.Equal(msg.Twist.Twist.Angular.Z, deserializedMsg.Twist.Twist.Angular.Z);
            Assert.True(msg.Twist.Covariance.SequenceEqual(deserializedMsg.Twist.Covariance));
        }
        finally
        {
            native.Dispose();
        }
    }
}
