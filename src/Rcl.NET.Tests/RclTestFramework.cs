using System.Reflection;
using Xunit.Abstractions;
using Xunit.Sdk;

namespace Rcl.NET.Tests;
public class RclTestFramework : XunitTestFramework
{
    public RclTestFramework(IMessageSink messageSink)
        : base(messageSink)
    {
        messageSink.OnMessage(new DiagnosticMessage($"Running tests with {nameof(RclTestFramework)}, ProcessorCount = {Environment.ProcessorCount}."));
    }

    protected override ITestFrameworkExecutor CreateExecutor(AssemblyName assemblyName)
        => new CustomExecutor(assemblyName, SourceInformationProvider, DiagnosticMessageSink);

    private class CustomExecutor : XunitTestFrameworkExecutor
    {
        public CustomExecutor(AssemblyName assemblyName, ISourceInformationProvider sourceInformationProvider, IMessageSink diagnosticMessageSink)
            : base(assemblyName, sourceInformationProvider, diagnosticMessageSink)
        {
        }

        protected override async void RunTestCases(IEnumerable<IXunitTestCase> testCases, IMessageSink executionMessageSink, ITestFrameworkExecutionOptions executionOptions)
        {
            using var assemblyRunner = new CustomAssemblyRunner(TestAssembly, testCases, DiagnosticMessageSink, executionMessageSink, executionOptions);
            await assemblyRunner.RunAsync();
        }
    }

    private class CustomAssemblyRunner : XunitTestAssemblyRunner
    {
        public CustomAssemblyRunner(ITestAssembly testAssembly, IEnumerable<IXunitTestCase> testCases, IMessageSink diagnosticMessageSink, IMessageSink executionMessageSink, ITestFrameworkExecutionOptions executionOptions)
            : base(testAssembly, testCases, diagnosticMessageSink, executionMessageSink, executionOptions)
        {
        }

        protected override Task<RunSummary> RunTestCollectionAsync(IMessageBus messageBus, ITestCollection testCollection, IEnumerable<IXunitTestCase> testCases, CancellationTokenSource cancellationTokenSource)
            => new CustomTestCollectionRunner(testCollection, testCases, DiagnosticMessageSink, messageBus, TestCaseOrderer, new ExceptionAggregator(Aggregator), cancellationTokenSource).RunAsync();
    }

    private class CustomTestCollectionRunner : XunitTestCollectionRunner
    {
        public CustomTestCollectionRunner(ITestCollection testCollection, IEnumerable<IXunitTestCase> testCases, IMessageSink diagnosticMessageSink, IMessageBus messageBus, ITestCaseOrderer testCaseOrderer, ExceptionAggregator aggregator, CancellationTokenSource cancellationTokenSource)
            : base(testCollection, testCases, diagnosticMessageSink, messageBus, testCaseOrderer, aggregator, cancellationTokenSource)
        {
        }

        protected override Task<RunSummary> RunTestClassAsync(ITestClass testClass, IReflectionTypeInfo @class, IEnumerable<IXunitTestCase> testCases)
            => new CustomTestClassRunner(testClass, @class, testCases, DiagnosticMessageSink, MessageBus, TestCaseOrderer, new ExceptionAggregator(Aggregator), CancellationTokenSource, CollectionFixtureMappings)
                .RunAsync();
    }

    private class CustomTestClassRunner : XunitTestClassRunner
    {
        public CustomTestClassRunner(ITestClass testClass, IReflectionTypeInfo @class, IEnumerable<IXunitTestCase> testCases, IMessageSink diagnosticMessageSink, IMessageBus messageBus, ITestCaseOrderer testCaseOrderer, ExceptionAggregator aggregator, CancellationTokenSource cancellationTokenSource, IDictionary<Type, object> collectionFixtureMappings)
            : base(testClass, @class, testCases, diagnosticMessageSink, messageBus, testCaseOrderer, aggregator, cancellationTokenSource, collectionFixtureMappings)
        {
        }

        protected override Task<RunSummary> RunTestMethodAsync(ITestMethod testMethod, IReflectionMethodInfo method, IEnumerable<IXunitTestCase> testCases, object[] constructorArguments)
            => new CustomTestMethodRunner(testMethod, Class, method, testCases, DiagnosticMessageSink, MessageBus, new ExceptionAggregator(Aggregator), CancellationTokenSource, constructorArguments)
                .RunAsync();
    }

    private class CustomTestMethodRunner : XunitTestMethodRunner
    {
        private readonly object[] _constructorArguments;

        public CustomTestMethodRunner(ITestMethod testMethod, IReflectionTypeInfo @class, IReflectionMethodInfo method, IEnumerable<IXunitTestCase> testCases, IMessageSink diagnosticMessageSink, IMessageBus messageBus, ExceptionAggregator aggregator, CancellationTokenSource cancellationTokenSource, object[] constructorArguments)
            : base(testMethod, @class, method, testCases, diagnosticMessageSink, messageBus, aggregator, cancellationTokenSource, constructorArguments)
        {
            _constructorArguments = constructorArguments;
        }

        protected override Task<RunSummary> RunTestCaseAsync(IXunitTestCase testCase)
        {
            // The tested Windows Foxy binaries use Cyclone DDS 0.7.0. Reusing a thread slot resets its
            // vtime while a pending GC request can still hold the old, larger value. GC then
            // stops progressing and dds_reader_close waits indefinitely during subscription fini.
            // RclContext event-loop threads and async workers expose this across serial tests;
            // limiting concurrent test methods therefore does not prevent the hang.
            // A native dump showed an old vtime of 1185 versus 16 in an already vacant slot.
            // Preserving vtime in a diagnostic Cyclone build allowed the full suite to complete.
            // Reconsider this distro/RMW exclusion when the bundled Cyclone binary is upgraded.
            // Upstream fix:
            // https://github.com/eclipse-cyclonedds/cyclonedds/commit/9acd956b8797120247d2a54a2dddef95a2c48825
            if (OperatingSystem.IsWindows() && RosEnvironment.IsFoxy
                && RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp")
            {
                return new XunitTestCaseRunner(testCase, testCase.DisplayName,
                    "Windows Foxy / Cyclone DDS: upstream thread-state reuse can hang native teardown.",
                    _constructorArguments, testCase.TestMethodArguments, MessageBus,
                    new ExceptionAggregator(Aggregator), CancellationTokenSource).RunAsync();
            }

            return base.RunTestCaseAsync(testCase);
        }
    }
}
