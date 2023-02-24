using Rcl.Actions;
using Rcl.Actions.Client;
using Rcl.Actions.Server;
using Rcl.Graph;
using Rcl.Internal.Clients;
using Rcl.Internal.NodeServices;
using Rcl.Internal.Services;
using Rcl.Internal.Subscriptions;
using Rcl.Logging;
using Rcl.Parameters;
using Rcl.Parameters.Impl;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Text;
using System.Threading.Channels;

namespace Rcl.Internal;

partial class RclNodeImpl : RclObject<SafeNodeHandle>, IRclNode
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
        : base(new(context.Handle, name, @namespace, options ?? NodeOptions.Default))
    {
        Context = context;
        Options = options ?? NodeOptions.Default;
        Clock = Options.Clock;

        Name = StringMarshal.CreatePooledString(rcl_node_get_name(Handle.Object))!;
        Namespace = StringMarshal.CreatePooledString(rcl_node_get_namespace(Handle.Object))!;
        FullyQualifiedName = StringMarshal.CreatePooledString(rcl_node_get_fully_qualified_name(Handle.Object))!;
        Logger = context.LoggerFactory.CreateLogger(StringMarshal.CreatePooledString(rcl_node_get_logger_name(Handle.Object))!);

        _graph = new(this);
        var graphSignal = new RclGuardConditionImpl(context,
            new(rcl_node_get_graph_guard_condition(Handle.Object)));
        _ = GraphBuilder(graphSignal, _cts.Token);

        _parameters = new ParameterService(this, new());
        _timeSource = new ExternalTimeSource(this, Options.ClockQoS);
    }

    public IParameterService Parameters => _parameters;

    public RclClock Clock { get; }

    public NodeOptions Options { get; }

    public RclContext Context { get; }

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
        await Context.Yield();

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
    }
}