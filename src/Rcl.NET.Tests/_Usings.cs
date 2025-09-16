global using Xunit;
using System.Runtime.CompilerServices;

[assembly: DisableRuntimeMarshalling]
[assembly: TestFramework("Rcl.NET.Tests.RclTestFramework", "Rcl.NET.Tests")]
[assembly: CollectionBehavior(CollectionBehavior.CollectionPerAssembly, DisableTestParallelization = true)]