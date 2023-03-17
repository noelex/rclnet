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
    private readonly ExternalTimeSource _timeSource;
    private readonly ParameterService _parameters;
    private readonly CancellationTokenSource _cts = new();
    private readonly RclGuardConditionImpl _graphSignal;

    public unsafe RclNodeImpl(
        RclContext context,
        string name,
         string @namespace = "",
        NodeOptions? options = null)
        : base(context, new(context.Handle, name, @namespace, options ?? NodeOptions.Default))
    {
        Options = options ?? NodeOptions.Default;
        Clock = Options.Clock switch
        {
            RclClockType.Ros => new(context, Options.Clock),
            RclClockType.Steady => context.SteadyClock,
            RclClockType.System => context.SystemClock,
            _ => throw new RclException($"Unsupported clock type '{Options.Clock}'.")
        };

        Name = StringMarshal.CreatePooledString(rcl_node_get_name(Handle.Object))!;
        Namespace = StringMarshal.CreatePooledString(rcl_node_get_namespace(Handle.Object))!;
        FullyQualifiedName = StringMarshal.CreatePooledString(rcl_node_get_fully_qualified_name(Handle.Object))!;
        Logger = context.CreateLogger(StringMarshal.CreatePooledString(rcl_node_get_logger_name(Handle.Object))!);

        _graph = new(this);
        _graphSignal = new RclGuardConditionImpl(context,
            new(rcl_node_get_graph_guard_condition(Handle.Object)));
        _ = GraphBuilder(_graphSignal, _cts.Token);

        var overrides = Options.ParameterOverrides ?? s_emptyParameterOverrides;
        _parameters = new ParameterService(this, overrides);
        _timeSource = new ExternalTimeSource(this, Options.ClockQos);

        if (Options.DeclareParameterFromOverrides)
        {
            foreach(var (k,v) in overrides)
            {
                _parameters.Declare(k, v);
            }
        }
    }

    public IParameterService Parameters => _parameters;

    public RclClock Clock { get; }

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
        _timeSource.Dispose();
        _parameters.Dispose();
        _cts.Cancel();
        _cts.Dispose();

        // Graph signal must be disposed before node because the graph signal is owned by the node handle.
        // When the node is diposed, the graph signal is also disposed internally, which is invisible to us.
        // This will cause segfault when the event loop tries to wait for the already disposed graph signal.
        //
        // It's safe to dispose the signal here as the diposal happens on the event loop internally.
        _graphSignal.Dispose();
        base.Dispose();

        if (Clock.Type == RclClockType.Ros) Clock.Dispose();
    }
}