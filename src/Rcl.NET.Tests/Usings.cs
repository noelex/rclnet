global using Xunit;
using System.Runtime.CompilerServices;

[assembly: DisableRuntimeMarshalling]
[assembly: CollectionBehavior(DisableTestParallelization = true)]
[assembly: TestFramework("Rcl.NET.Tests.RclTestFramework", "Rcl.NET.Tests")]