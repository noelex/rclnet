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
    private readonly RosGraph _graph;
    private readonly ExternalTimeSource _timeSource;
    private readonly ParameterService _parameters;
    private readonly CancellationTokenSource _cts = new();

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
            RclClockType.Ros => new(Options.Clock),
            RclClockType.Steady => RclClock.Steady,
            RclClockType.System => RclClock.System,
            _ => throw new RclException($"Unsupported clock type '{Options.Clock}'.")
        };

        Name = StringMarshal.CreatePooledString(rcl_node_get_name(Handle.Object))!;
        Namespace = StringMarshal.CreatePooledString(rcl_node_get_namespace(Handle.Object))!;
        FullyQualifiedName = StringMarshal.CreatePooledString(rcl_node_get_fully_qualified_name(Handle.Object))!;
        Logger = context.CreateLogger(StringMarshal.CreatePooledString(rcl_node_get_logger_name(Handle.Object))!);

        _graph = new(this);
        var graphSignal = new RclGuardConditionImpl(context,
            new(rcl_node_get_graph_guard_condition(Handle.Object)));
        _ = GraphBuilder(graphSignal, _cts.Token);

        _parameters = new ParameterService(this, new());
        _timeSource = new ExternalTimeSource(this, Options.ClockQos);
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

        using (graphSignal)
        {
            try
            {
                while (!cancellationToken.IsCancellationRequested)
                {
                    try
                    {
                        _graph.Build();
                    }
                    catch (Exception e)
                    {
                        Console.WriteLine("Unable to build ROS graph: " + e.Message);
                        Console.WriteLine(e.StackTrace);
                    }

                    await graphSignal.WaitOneAsync(cancellationToken).ConfigureAwait(false);
                }
            }
            finally
            {
                _graph.Complete();
            }
        }
    }

    public override void Dispose()
    {
        _timeSource.Dispose();
        _parameters.Dispose();
        _cts.Cancel();
        _cts.Dispose();
        base.Dispose();
        if (!Clock.IsShared) Clock.Dispose();
    }
}