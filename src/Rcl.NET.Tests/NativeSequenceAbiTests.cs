using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.NET.Tests;

public class NativeSequenceAbiTests
{
    [Fact]
    public unsafe void WrongAbiSequencesRejectNativeCalls()
    {
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V2)
        {
            Assert.Throws<InvalidOperationException>(() => new UInt8Sequence(1));
            Assert.Throws<InvalidOperationException>(() => new CStringSequence(1));
            Assert.Throws<InvalidOperationException>(() => CStringSequence.Create(1));
            Assert.Throws<InvalidOperationException>(() => CStringSequence.Destroy(null));

            var sequence = default(UInt8Sequence);
            Assert.Throws<InvalidOperationException>(() => sequence.CopyFrom(sequence));
            Assert.Throws<InvalidOperationException>(() => sequence.Equals(sequence));
            Assert.Throws<InvalidOperationException>(() => sequence.Dispose());
        }
        else
        {
            Assert.Throws<InvalidOperationException>(() => new UInt8SequenceV2(1));
            Assert.Throws<InvalidOperationException>(() => new CStringSequenceV2(1));
            Assert.Throws<InvalidOperationException>(() => CStringSequenceV2.Create(1));
            Assert.Throws<InvalidOperationException>(() => CStringSequenceV2.Destroy(null));

            var sequence = default(UInt8SequenceV2);
            Assert.Throws<InvalidOperationException>(() => sequence.CopyFrom(sequence));
            Assert.Throws<InvalidOperationException>(() => sequence.Equals(sequence));
            Assert.Throws<InvalidOperationException>(() => sequence.Dispose());
        }
    }
}
