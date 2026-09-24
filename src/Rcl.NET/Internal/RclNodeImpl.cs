using Rcl.Graph;
using Rcl.Internal.NodeServices;
using Rcl.Logging;
using Rcl.Parameters;
using Rcl.Parameters.Impl;
using Rcl.SafeHandles;
using Rosidl.Runtime.Interop;

namespace Rcl.Internal;

partial class RclNodeImpl : RclContextualObject<SafeNodeHandle>, IRclNode
{
    private static readonly Dictionary<string, Variant> s_emptyParameterOverrides = new();

    private readonly RosGraph _graph;
    private readonly ExternalTimeSource? _timeSource;
    private readonly ParameterService _parameters;
    private readonly RclTimeProvider _timeProvider;
    private readonly CancellationTokenSource _cts = new();
    private readonly RclGuardConditionImpl _graphSignal;
    private readonly bool _ownsClock;
    private int _disposed;

    public unsafe RclNodeImpl(
        RclContext context,
        string name,
        string @namespace = "",
        RclClock? clockOverride = null,
        NodeOptions? options = null)
        : base(context, new(context.Handle, name, @namespace, options ?? NodeOptions.Default))
    {
        try
        {
            Options = options ?? NodeOptions.Default;
            _ownsClock = clockOverride == null && Options.Clock == RclClockType.Ros;
            Clock = clockOverride ?? Options.Clock switch
            {
                RclClockType.Ros => new(Options.Clock),
                RclClockType.Steady => RclClock.SteadyClock,
                RclClockType.System => RclClock.SystemClock,
                _ => throw new RclException($"Unsupported clock type '{Options.Clock}'.")
            };
            _timeProvider = new(context, Clock);

            Name = StringMarshal.CreatePooledString(rcl_node_get_name(Handle.Object))!;
            Namespace = StringMarshal.CreatePooledString(rcl_node_get_namespace(Handle.Object))!;
            FullyQualifiedName = StringMarshal.CreatePooledString(rcl_node_get_fully_qualified_name(Handle.Object))!;
            Logger = context.CreateLogger(StringMarshal.CreatePooledString(rcl_node_get_logger_name(Handle.Object))!);

            _graph = new(this, Options.GraphEventFilter ?? (static _ => true));
            _graphSignal = new RclGuardConditionImpl(context,
                SafeGuardConditionHandle.BorrowGraphGuard(Handle));

            var overrides = Options.ParameterOverrides ?? s_emptyParameterOverrides;
            _parameters = new ParameterService(this, overrides);

            // Create the time source only when we're not using clockOverride.
            if (clockOverride == null)
            {
                _timeSource = new ExternalTimeSource(this, Options.ClockQos);
            }

            if (Options.DeclareParameterFromOverrides)
            {
                foreach (var (k, v) in overrides)
                {
                    _parameters.Declare(k, v);
                }
            }
            Handle.ThrowIfDescendantClosed();
            _ = GraphBuilder(_graphSignal, _cts.Token);
        }
        catch { Dispose(); throw; }
    }

    public IParameterService Parameters => _parameters;

    public RclClock Clock { get; }

    public TimeProvider TimeProvider => _timeProvider;

    public NodeOptions Options { get; }

    public RosGraph Graph => _graph;

    public IRclLogger Logger { get; }

    public string Name { get; }

    public string Namespace { get; }

    public string FullyQualifiedName { get; }

    public unsafe ulong InstanceId
         => rcl_node_get_rcl_instance_id(Handle.Object);

    public unsafe bool IsValid
         => rcl_node_is_valid(Handle.Object);

    public unsafe nuint DomaindId
    {
        get
        {
            size_t s;
            rcl_node_get_domain_id(Handle.Object, &s);
            return s;
        }
    }

    IRclContext IRclNode.Context => Context;

    IRclClock IRclNode.Clock => Clock;

    private async Task GraphBuilder(RclGuardConditionImpl graphSignal, CancellationToken cancellationToken)
    {
        await Context.YieldIfNotCurrent();

        try
        {
            while (true)
            {
                try
                {
                    _graph.Build();
                }
                catch (Exception e)
                {
                    Logger.LogWarning("Unable to build ROS graph: " + e.Message);
                    Logger.LogWarning(e.StackTrace);
                }

                await graphSignal.WaitOneAsync(false, cancellationToken).ConfigureAwait(false);
            }
        }
        finally
        {
            _graph.Complete();
        }
    }

    public override void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0) return;
        Handle.TryBeginClose();
        try
        {
            try { _timeProvider?.Dispose(); }
            finally
            {
                try { _timeSource?.Dispose(); }
                finally { _parameters?.Dispose(); }
            }
        }
        finally
        {
            _cts.Cancel();
            _cts.Dispose();
            _graphSignal?.Dispose();
            base.Dispose();
            if (_ownsClock && Clock != null)
                Context.SynchronizationContext.Post(static x => ((IDisposable)x!).Dispose(), Clock);
        }
    }
}
