using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.NET.Tests;
internal class NameGenerator
{
    private static long _id;

    public static string GenerateNodeName() => $"test_node_{Interlocked.Increment(ref _id)}";

    public static string GenerateTopicName() => $"test_topic_{Interlocked.Increment(ref _id)}";

    public static string GenerateServiceName() => $"test_service_{Interlocked.Increment(ref _id)}";
}
