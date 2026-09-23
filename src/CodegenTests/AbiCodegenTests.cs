using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Reflection;
using System.Runtime.CompilerServices;
using Xunit;
using Portable = Rosidl.Test.Portable.Ros2csAbiTest;
using V1 = Rosidl.Test.V1.Ros2csAbiTest;
using V2 = Rosidl.Test.V2.Ros2csAbiTest;

namespace CodegenTests;

[Collection(RosEnvironmentTestCollection.Name)]
public class AbiCodegenTests
{
    [Fact]
    public void FixedModesEmitOneNativeLayout()
    {
        AssertLayoutSet(typeof(V1.Scalar), ("Priv", RosidlNativeAbi.V1), ("PrivSequence", RosidlNativeAbi.V1));
        AssertLayoutSet(typeof(V2.Scalar), ("Priv", RosidlNativeAbi.V2), ("PrivSequence", RosidlNativeAbi.V2));

        AssertNativeCodec(typeof(V1.Scalar), typeof(V1.Scalar.Priv));
        AssertNativeCodec(typeof(V2.Scalar), typeof(V2.Scalar.Priv));
    }

    [Fact]
    public void PortableModeEmitsBothNativeLayouts()
    {
        AssertLayoutSet(
            typeof(Portable.Scalar),
            ("Priv", RosidlNativeAbi.V1),
            ("PrivSequence", RosidlNativeAbi.V1),
            ("PrivV2", RosidlNativeAbi.V2),
            ("PrivSequenceV2", RosidlNativeAbi.V2));

        AssertNativeCodec(typeof(Portable.Scalar), typeof(Portable.Scalar.Priv));
        AssertNativeCodec(typeof(Portable.Scalar), typeof(Portable.Scalar.PrivV2));
    }

    [Fact]
    public void NativeLayoutsExposeGenericContracts()
    {
        AssertNativeType<V1.Scalar.Priv>(RosidlNativeAbi.V1);
        AssertNativeSequence<V1.Scalar.PrivSequence, V1.Scalar.Priv>(RosidlNativeAbi.V1);
        AssertNativeType<V2.Scalar.Priv>(RosidlNativeAbi.V2);
        AssertNativeSequence<V2.Scalar.PrivSequence, V2.Scalar.Priv>(RosidlNativeAbi.V2);
        AssertNativeType<Portable.Scalar.Priv>(RosidlNativeAbi.V1);
        AssertNativeSequence<Portable.Scalar.PrivSequence, Portable.Scalar.Priv>(RosidlNativeAbi.V1);
        AssertNativeType<Portable.Scalar.PrivV2>(RosidlNativeAbi.V2);
        AssertNativeSequence<Portable.Scalar.PrivSequenceV2, Portable.Scalar.PrivV2>(RosidlNativeAbi.V2);
    }

    [Fact]
    public void SequenceLayoutsPreserveFollowingFieldOffsets()
    {
        var v1 = default(V1.PrimitiveSequence.Priv);
        var v2 = default(V2.PrimitiveSequence.Priv);
        var portableV1 = default(Portable.PrimitiveSequence.Priv);
        var portableV2 = default(Portable.PrimitiveSequence.PrivV2);
        var v1SequenceSize = Unsafe.SizeOf<UInt8Sequence>();
        var v2SequenceSize = Unsafe.SizeOf<UInt8SequenceV2>();

        Assert.Equal(v1SequenceSize + 8, Unsafe.SizeOf<V1.PrimitiveSequence.Priv>());
        Assert.Equal(v2SequenceSize + 8, Unsafe.SizeOf<V2.PrimitiveSequence.Priv>());
        Assert.Equal(v1SequenceSize, OffsetOf(ref v1, ref v1.TrailingBool));
        Assert.Equal(v1SequenceSize + 4, OffsetOf(ref v1, ref v1.TrailingValue));
        Assert.Equal(v2SequenceSize, OffsetOf(ref v2, ref v2.TrailingBool));
        Assert.Equal(v2SequenceSize + 4, OffsetOf(ref v2, ref v2.TrailingValue));

        Assert.Equal(Unsafe.SizeOf<V1.PrimitiveSequence.Priv>(), Unsafe.SizeOf<Portable.PrimitiveSequence.Priv>());
        Assert.Equal(Unsafe.SizeOf<V2.PrimitiveSequence.Priv>(), Unsafe.SizeOf<Portable.PrimitiveSequence.PrivV2>());
        Assert.Equal(v1SequenceSize, OffsetOf(ref portableV1, ref portableV1.TrailingBool));
        Assert.Equal(v2SequenceSize, OffsetOf(ref portableV2, ref portableV2.TrailingBool));
    }

    [Fact]
    public void MessageSequenceLayoutsMatchTheirAbi()
    {
        Assert.Equal(3 * IntPtr.Size, Unsafe.SizeOf<V1.Scalar.PrivSequence>());
        Assert.Equal(3 * IntPtr.Size, Unsafe.SizeOf<V2.Scalar.PrivSequence>());
        Assert.Equal(3 * IntPtr.Size, Unsafe.SizeOf<Portable.Scalar.PrivSequence>());
        Assert.Equal(3 * IntPtr.Size, Unsafe.SizeOf<Portable.Scalar.PrivSequenceV2>());
    }

    [Fact]
    public void NestedGraphsUseOneLayoutThroughout()
    {
        AssertFieldType(typeof(V1.Nested.Priv), "Value", typeof(V1.PrimitiveSequence.Priv));
        AssertFieldType(typeof(V2.Nested.Priv), "Value", typeof(V2.PrimitiveSequence.Priv));
        AssertFieldType(typeof(Portable.Nested.Priv), "Value", typeof(Portable.PrimitiveSequence.Priv));
        AssertFieldType(typeof(Portable.Nested.PrivV2), "Value", typeof(Portable.PrimitiveSequence.PrivV2));

        AssertFieldType(typeof(V1.ComplexSequence.Priv), "Values", typeof(V1.Nested.PrivSequence));
        AssertFieldType(typeof(V2.ComplexSequence.Priv), "Values", typeof(V2.Nested.PrivSequence));
        AssertFieldType(typeof(Portable.ComplexSequence.Priv), "Values", typeof(Portable.Nested.PrivSequence));
        AssertFieldType(typeof(Portable.ComplexSequence.PrivV2), "Values", typeof(Portable.Nested.PrivSequenceV2));

        AssertFieldType(typeof(V1.FixedComplexArray.Priv), "__Values_0", typeof(V1.Nested.Priv), nonPublic: true);
        AssertFieldType(typeof(V2.FixedComplexArray.Priv), "__Values_0", typeof(V2.Nested.Priv), nonPublic: true);
        AssertFieldType(typeof(Portable.FixedComplexArray.Priv), "__Values_0", typeof(Portable.Nested.Priv), nonPublic: true);
        AssertFieldType(typeof(Portable.FixedComplexArray.PrivV2), "__Values_0", typeof(Portable.Nested.PrivV2), nonPublic: true);
    }

    [Fact]
    public void PrimitiveAndStringSequencesUseMatchingRuntimeTypes()
    {
        AssertFieldType(typeof(V1.PrimitiveSequence.Priv), "Data", typeof(UInt8Sequence));
        AssertFieldType(typeof(V2.PrimitiveSequence.Priv), "Data", typeof(UInt8SequenceV2));
        AssertFieldType(typeof(Portable.PrimitiveSequence.Priv), "Data", typeof(UInt8Sequence));
        AssertFieldType(typeof(Portable.PrimitiveSequence.PrivV2), "Data", typeof(UInt8SequenceV2));

        AssertFieldType(typeof(V1.StringSequence.Priv), "Values", typeof(CStringSequence));
        AssertFieldType(typeof(V2.StringSequence.Priv), "Values", typeof(CStringSequenceV2));
        AssertFieldType(typeof(Portable.StringSequence.Priv), "Values", typeof(CStringSequence));
        AssertFieldType(typeof(Portable.StringSequence.PrivV2), "Values", typeof(CStringSequenceV2));
    }

    [Fact]
    public void ServiceAndActionMessagesFollowTheSelectedMode()
    {
        AssertLayoutSet(
            typeof(Portable.SequenceServiceServiceRequest),
            ("Priv", RosidlNativeAbi.V1),
            ("PrivSequence", RosidlNativeAbi.V1),
            ("PrivV2", RosidlNativeAbi.V2),
            ("PrivSequenceV2", RosidlNativeAbi.V2));
        AssertLayoutSet(
            typeof(Portable.SequenceActionFeedback),
            ("Priv", RosidlNativeAbi.V1),
            ("PrivSequence", RosidlNativeAbi.V1),
            ("PrivV2", RosidlNativeAbi.V2),
            ("PrivSequenceV2", RosidlNativeAbi.V2));

        AssertFieldType(typeof(V2.SequenceServiceServiceRequest.Priv), "Values", typeof(UInt8SequenceV2));
        AssertFieldType(typeof(V2.SequenceActionResult.Priv), "ResultValues", typeof(CStringSequenceV2));
        AssertFieldType(
            typeof(Portable.SequenceActionFeedback.PrivV2),
            "FeedbackValues",
            typeof(Portable.Nested.PrivSequenceV2));
    }

    [Fact]
    public void NativeTypeNameCollisionsAreEscaped()
    {
        var messageType = typeof(Portable.NameCollision);

        Assert.NotNull(messageType.GetProperty("Priv_"));
        Assert.NotNull(messageType.GetProperty("PrivV2_"));
        Assert.NotNull(messageType.GetProperty("PrivSequence_"));
        Assert.NotNull(messageType.GetProperty("PrivSequenceV2_"));
        AssertFieldType(typeof(Portable.NameCollision.Priv), "PrivV2_", typeof(int));
        AssertFieldType(typeof(Portable.NameCollision.PrivV2), "PrivV2_", typeof(int));
    }

    [Fact]
    public void PortableWrongAbiLifecycleIsRejectedBeforeNativeCall()
    {
        var originalDistro = Environment.GetEnvironmentVariable("ROS_DISTRO");

        try
        {
            Environment.SetEnvironmentVariable("ROS_DISTRO", "kilted");

            var exception = Assert.Throws<InvalidOperationException>(
                () => Portable.Scalar.PrivV2.TryInitialize(out _));
            Assert.Contains("V2", exception.Message);
            Assert.Contains("V1", exception.Message);
        }
        finally
        {
            Environment.SetEnvironmentVariable("ROS_DISTRO", originalDistro);
        }
    }

    private static void AssertLayoutSet(Type messageType, params (string Name, RosidlNativeAbi Abi)[] expected)
    {
        var actual = messageType
            .GetNestedTypes(BindingFlags.Public)
            .Where(x => x.Name.StartsWith("Priv", StringComparison.Ordinal))
            .OrderBy(x => x.Name)
            .ToArray();

        Assert.Equal(expected.Select(x => x.Name).OrderBy(x => x), actual.Select(x => x.Name));
        foreach (var (name, abi) in expected)
        {
            var nativeType = Assert.Single(actual, x => x.Name == name);
            Assert.Equal(abi, nativeType.GetCustomAttribute<RosidlAbiAttribute>()?.Abi);
        }
    }

    private static void AssertNativeCodec(Type messageType, Type nativeType)
    {
        var parameters = new[] { nativeType.MakeByRefType(), typeof(System.Text.Encoding) };

        Assert.NotNull(messageType.GetConstructor(parameters));
        Assert.NotNull(messageType.GetMethod("WriteTo", parameters));
    }

    private static void AssertNativeType<T>(RosidlNativeAbi expectedAbi)
        where T : unmanaged, IRosidlNative
    {
        Assert.Equal(expectedAbi, T.Abi);
    }

    private static void AssertNativeSequence<TSequence, T>(RosidlNativeAbi expectedAbi)
        where TSequence : unmanaged, IRosidlNativeSequence<T>
        where T : unmanaged, IRosidlNative
    {
        var sequence = default(TSequence);

        Assert.Equal(expectedAbi, TSequence.Abi);
        Assert.Equal(0, sequence.Size);
    }

    private static void AssertFieldType(Type declaringType, string name, Type expectedType, bool nonPublic = false)
    {
        var flags = BindingFlags.Instance | (nonPublic ? BindingFlags.NonPublic : BindingFlags.Public);
        var field = declaringType.GetField(name, flags);

        Assert.NotNull(field);
        Assert.Equal(expectedType, field.FieldType);
    }

    private static int OffsetOf<T, TField>(ref T value, ref TField field)
    {
        ref var start = ref Unsafe.As<T, byte>(ref value);
        ref var target = ref Unsafe.As<TField, byte>(ref field);
        return checked((int)Unsafe.ByteOffset(ref start, ref target));
    }

}
