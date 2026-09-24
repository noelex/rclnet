global using Xunit;
using System.Runtime.CompilerServices;

[assembly: DisableRuntimeMarshalling]
[assembly: TestFramework("Rcl.NET.Tests.RclTestFramework", "Rcl.NET.Tests")]
// Tests share /clock, native logging configuration and release diagnostics.
// Exercise concurrency inside individual tests without racing these process-wide fixtures.
[assembly: CollectionBehavior(CollectionBehavior.CollectionPerAssembly, DisableTestParallelization = true)]
