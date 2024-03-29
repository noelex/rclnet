﻿using System.Reflection;
using System.Runtime.InteropServices;
using Xunit.Abstractions;
using Xunit.Sdk;

namespace Rcl.NET.Tests;
public class RclTestFramework : XunitTestFramework
{
    private static readonly SemaphoreSlim s_rateLimiter = new(1);
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

        protected override async Task<RunSummary> RunTestMethodAsync(ITestMethod testMethod, IReflectionMethodInfo method, IEnumerable<IXunitTestCase> testCases, object[] constructorArguments)
        {
            // Manually limiting concurrency here because Creating and disposing multiple RclContexts
            // concurrently may hang the test host when running on Windows with rmw_cyclonedds_cpp on foxy.
            if (RosEnvironment.IsFoxy &&
                RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp" &&
                OperatingSystem.IsWindows())
            {
                await s_rateLimiter.WaitAsync();

                try
                {
                    return await new CustomTestMethodRunner(testMethod, this.Class, method, testCases, this.DiagnosticMessageSink, this.MessageBus, new ExceptionAggregator(this.Aggregator), this.CancellationTokenSource, constructorArguments)
                         .RunAsync();
                }
                finally
                {
                    s_rateLimiter.Release();
                }
            }
            else
            {
                return await new CustomTestMethodRunner(testMethod, this.Class, method, testCases, this.DiagnosticMessageSink, this.MessageBus, new ExceptionAggregator(this.Aggregator), this.CancellationTokenSource, constructorArguments)
                         .RunAsync();
            }
        }
    }

    private class CustomTestMethodRunner : XunitTestMethodRunner
    {
        private readonly IMessageSink _diagnosticMessageSink;

        public CustomTestMethodRunner(ITestMethod testMethod, IReflectionTypeInfo @class, IReflectionMethodInfo method, IEnumerable<IXunitTestCase> testCases, IMessageSink diagnosticMessageSink, IMessageBus messageBus, ExceptionAggregator aggregator, CancellationTokenSource cancellationTokenSource, object[] constructorArguments)
            : base(testMethod, @class, method, testCases, diagnosticMessageSink, messageBus, aggregator, cancellationTokenSource, constructorArguments)
        {
            _diagnosticMessageSink = diagnosticMessageSink;
        }

        protected override Task<RunSummary> RunTestCaseAsync(IXunitTestCase testCase)
        {
            return base.RunTestCaseAsync(testCase);
            //var parameters = string.Empty;

            //if (testCase.TestMethodArguments != null)
            //{
            //    parameters = string.Join(", ", testCase.TestMethodArguments.Select(a => a?.ToString() ?? "null"));
            //}

            //var test = $"{TestMethod.TestClass.Class.Name}.{TestMethod.Method.Name}({parameters})";

            //_diagnosticMessageSink.OnMessage(new DiagnosticMessage($"STARTED: {test}"));

            //try
            //{
            //    var deadlineMinutes = 1;
            //    using var timer = new Timer(
            //        _ => _diagnosticMessageSink.OnMessage(new DiagnosticMessage($"WARNING: {test} has been running for more than {deadlineMinutes} minutes")),
            //        null,
            //        TimeSpan.FromMinutes(deadlineMinutes),
            //        Timeout.InfiniteTimeSpan);

            //    var result = await base.RunTestCaseAsync(testCase);

            //    var status = result.Failed > 0
            //        ? "FAILURE"
            //        : (result.Skipped > 0 ? "SKIPPED" : "SUCCESS");

            //    _diagnosticMessageSink.OnMessage(new DiagnosticMessage($"{status}: {test} ({result.Time}s)"));

            //    return result;
            //}
            //catch (Exception ex)
            //{
            //    _diagnosticMessageSink.OnMessage(new DiagnosticMessage($"ERROR: {test} ({ex.Message})"));
            //    throw;
            //}
        }
    }
}