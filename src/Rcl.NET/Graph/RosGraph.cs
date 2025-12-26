using Microsoft.Toolkit.HighPerformance.Buffers;
using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime.Interop;
using System.Collections;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;

namespace Rcl.Graph;

/// <summary>
/// Represents a graph of ROS communication objects.
/// </summary>
/// <remarks>
/// <see cref="RosGraph"/> allows user to query ROS graph and monitor graph changes in an
/// object-oriented and <see langword="async"/>/<see langword="await"/> friendly way.
/// <para>
/// All public properities and methods of a <see cref="RosGraph"/> object are thread-safe, which
/// means querying the state of the graph and monitoring graph events concurrently from many threads
/// are allowed.
/// </para>
/// <para>
/// Though concurrent access to the <see cref="RosGraph"/> object will never corrupt its internal state,
/// but if a user attempts to query the graph from another thread while
/// <see cref="RosGraph"/> is still building, inconsistent results might be returned.
/// </para>
/// <para>
/// For retrieving consistent results, you can access the <see cref="RosGraph"/> object from
/// the <see cref="RclContext"/> which the <see cref="RosGraph"/> object belongs (e.g. by yielding
/// to the <see cref="RclContext"/> using <c><see langword="await"/> graph.Owner.Context.Yield()</c>).
/// </para>
/// </remarks>
public partial class RosGraph : IGraphBuilder, IObservable<RosGraphEvent>
{
    private long _subscriberId;
    private readonly RclNodeImpl _node;

    private readonly ConcurrentDictionary<long, IObserver<RosGraphEvent>> _observers = new();

    private readonly ConcurrentDictionary<NodeName, RosNode> _nodes = new();
    private readonly ConcurrentDictionary<string, RosTopic> _topics = new();
    private readonly ConcurrentDictionary<string, RosService> _services = new();

    private readonly Dictionary<RosNode, PoolingList<RosTopicEndPoint>>
        _totalPublications = new(), _totalSubscriptions = new();
    private readonly Dictionary<RosService, PoolingList<RosServiceEndPoint>>
        _totalServers = new(), _totalClients = new();

    private readonly List<RosNode>
        _newNodes = new(), _removedNodes = new();
    private readonly List<RosService>
        _newServices = new(), _removedServices = new();
    private readonly List<RosServiceEndPoint>
        _newServers = new(), _removedServers = new(),
        _newClients = new(), _removedClients = new();
    private readonly List<RosTopic>
        _newTopics = new(), _removedTopics = new();
    private readonly List<RosTopicEndPoint>
        _newPublishers = new(), _removedPublishers = new(),
        _newSubscribers = new(), _removedSubscribers = new();

    private readonly IList[] _cleanupTargets;
    private readonly Func<NodeName, bool> _nodeFilter;

    internal RosGraph(RclNodeImpl node, Func<NodeName, bool> nodeFilter)
    {
        _node = node;
        _cleanupTargets = new IList[]
        {
            _newNodes,_removedNodes,

            _newServices,_removedServices,
            _newServers,_removedServers,
            _newClients,_removedClients,

            _newTopics,_removedTopics,
            _newPublishers,_removedPublishers,
            _newSubscribers,_removedSubscribers,

            _newActions, _removedActions,
            _totalActionClients, _newActionClients, _removedActionClients,
            _totalActionServers, _newActionServers, _removedActionServers
        };

        _nodeFilter = nodeFilter;
    }

    /// <summary>
    /// Owner node of current <see cref="RosGraph"/> instance.
    /// </summary>
    public IRclNode Owner => _node;

    /// <summary>
    /// Returns nodes currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosNode> Nodes => (IReadOnlyCollection<RosNode>)_nodes.Values;

    /// <summary>
    /// Returns topics currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosTopic> Topics => (IReadOnlyCollection<RosTopic>)_topics.Values;

    /// <summary>
    /// Returns services currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosService> Services => (IReadOnlyCollection<RosService>)_services.Values;

    /// <summary>
    /// Returns actions currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosAction> Actions => (IReadOnlyCollection<RosAction>)_actions.Values;

    internal void Build()
    {
        try
        {
            BuildNodes();
            BuildTopics(disableTopicNameDemangling: false);
            RelationshipFixup();
            BuildActions();

            FireEvents();
        }
        finally
        {
            Cleanup();
        }
    }

    private void RelationshipFixup()
    {
        foreach (var (node, endpoints) in _totalPublications)
        {
            node.ResetPublishers(endpoints.AsSpan());
        }
        foreach (var (node, endpoints) in _totalSubscriptions)
        {
            node.ResetSubscribers(endpoints.AsSpan());
        }

        foreach (var (svc, endpoints) in _totalServers)
        {
            svc.ResetServers(endpoints.AsSpan());
        }
        foreach (var (svc, endpoints) in _totalClients)
        {
            svc.ResetClients(endpoints.AsSpan());
        }

        foreach (var (k, v) in _services)
        {
            if (v.Clients.Count == 0 && v.Servers.Count == 0)
            {
                _services.Remove(k, out _);
                _removedServices.Add(v);
            }
        }
    }

    private void Cleanup()
    {
        foreach (var list in _cleanupTargets)
        {
            list.Clear();
        }

        foreach (var v in _totalPublications.Values) v.Dispose();
        _totalPublications.Clear();

        foreach (var v in _totalSubscriptions.Values) v.Dispose();
        _totalSubscriptions.Clear();

        foreach (var v in _totalServers.Values) v.Dispose();
        _totalServers.Clear();

        foreach (var v in _totalClients.Values) v.Dispose();
        _totalClients.Clear();
    }

    private unsafe void BuildNodes()
    {
        rcutils_string_array_t names, namespaces, enclaves;

        RclException.ThrowIfNonSuccess(
            rcl_get_node_names_with_enclaves(_node.Handle.Object,
                RclAllocator.Default.Object,
                &names,
                &namespaces,
                &enclaves));

        try
        {
            using var discoveredNodes = SpanOwner<NodeName>.Allocate((int)names.size.Value);

            for (var i = 0; i < (int)names.size.Value; i++)
            {
                var name = new NodeName(
                    StringMarshal.CreatePooledString((byte*)names.data[i])!,
                    StringMarshal.CreatePooledString((byte*)namespaces.data[i])!);

                if (_nodeFilter(name) && !_nodes.TryGetValue(name, out var node))
                {
                    var enclave = StringMarshal.CreatePooledString((byte*)enclaves.data[i])!;
                    _nodes[name] = node = new RosNode(name, enclave);
                    _newNodes.Add(node);
                }

                discoveredNodes.Span[i] = name;
            }

            foreach (var (k, node) in _nodes)
            {
                if (discoveredNodes.Span.IndexOf(k) < 0)
                {
                    if (_nodes.Remove(k, out var v))
                    {
                        _removedNodes.Add(v);

                        // Notify node to remove its service endpoints
                        // so that server and client events can be fired
                        // if the node disappears from graph.
                        v.UpdateServers(this, ReadOnlySpan<NameWithType>.Empty);
                        v.UpdateClients(this, ReadOnlySpan<NameWithType>.Empty);
                    }
                }
                else
                {
                    BuildNode(node);
                }
            }
        }
        finally
        {
            rcutils_string_array_fini(&names);
            rcutils_string_array_fini(&namespaces);
            rcutils_string_array_fini(&enclaves);
        }
    }

    private unsafe void BuildNode(RosNode node)
    {
        var nameSize = InteropHelpers.GetUtf8BufferSize(node.Name.Name);
        var nsSize = InteropHelpers.GetUtf8BufferSize(@node.Name.Namespace);
        Span<byte> nameBuffer = stackalloc byte[nameSize];
        Span<byte> nsBuffer = stackalloc byte[nsSize];
        InteropHelpers.FillUtf8Buffer(node.Name.Name, nameBuffer);
        InteropHelpers.FillUtf8Buffer(@node.Name.Namespace, nsBuffer);

        var namePtr = nameBuffer.IsEmpty ? null : (byte*)Unsafe.AsPointer(ref nameBuffer[0]);
        var nsPtr = nsBuffer.IsEmpty ? null : (byte*)Unsafe.AsPointer(ref nsBuffer[0]);

        FetchServiceEndpoints(node, namePtr, nsPtr);
    }

    private unsafe void BuildTopics(bool disableTopicNameDemangling = false)
    {
        var allocator = RclAllocator.Default.Object;
        rcl_names_and_types_t nts;

        RclException.ThrowIfNonSuccess(
            rcl_get_topic_names_and_types(
                _node.Handle.Object,
                &allocator,
                disableTopicNameDemangling,
                &nts));

        using var items = SpanOwner<NameWithType>.Allocate((int)nts.names.size.Value);
        CopyNameAndTypes(&nts, items.Span);

        foreach (var item in items.Span)
        {
            if (!_topics.TryGetValue(item.Name, out var topic))
            {
                _topics[item.Name] = topic = new(item.Name);
                _newTopics.Add(topic);
            }
        }

        foreach (var (k, v) in _topics)
        {
            var found = false;
            foreach (var item in items.Span)
            {
                if (item.Name == k)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                if (_topics.Remove(k, out var s))
                {
                    _removedTopics.Add(s);

                    // Notify topic to remove its topic endpoints
                    // so that publisher and subscriber events can be fired
                    // if the topic disappears from graph.
                    s.UpdatePublishers(this, ReadOnlySpan<TopicEndPointDataRef>.Empty, _nodes);
                    s.UpdateSubscribers(this, ReadOnlySpan<TopicEndPointDataRef>.Empty, _nodes);
                }
            }
            else
            {
                FetchTopicEndpoints(v, disableTopicNameDemangling);
            }
        }
    }

    private unsafe void FetchServiceEndpoints(RosNode node, byte* name, byte* ns)
    {
        var allocator = RclAllocator.Default.Object;
        rcl_names_and_types_t nts;

        var ret = rcl_get_service_names_and_types_by_node(
                _node.Handle.Object,
                &allocator,
                name,
                ns,
                &nts);

        // rcl_get_service_names_and_types_by_node may return RCL_RET_NODE_NAME_NON_EXISTENT
        // if the node gets removed from graph before we call this function.
        // We can continue as normal since nts will be empty and UpdateServers will see this
        // as all server endpoints getting removed from the node.
        // This also applies to the following rcl_get_client_names_and_types_by_node call.
        if (ret == rcl_ret_t.RCL_RET_NODE_NAME_NON_EXISTENT ||
            ret == rcl_ret_t.RCL_RET_OK)
        {
            using var items = SpanOwner<NameWithType>.Allocate((int)nts.names.size.Value);
            CopyNameAndTypes(&nts, items.Span);
            node.UpdateServers(this, items.Span);
        }
        else
        {
            RclException.ThrowIfNonSuccess(ret);
        }

        ret = rcl_get_client_names_and_types_by_node(
                  _node.Handle.Object,
                  &allocator,
                  name,
                  ns,
                  &nts);
        if (ret == rcl_ret_t.RCL_RET_NODE_NAME_NON_EXISTENT ||
            ret == rcl_ret_t.RCL_RET_OK)
        {
            using var items = SpanOwner<NameWithType>.Allocate((int)nts.names.size.Value);
            CopyNameAndTypes(&nts, items.Span);
            node.UpdateClients(this, items.Span);
        }
        else
        {
            RclException.ThrowIfNonSuccess(ret);
        }
    }

    private unsafe void FetchTopicEndpoints(RosTopic topic, bool disableTopicNameDemangling = false)
    {
        var nameSize = InteropHelpers.GetUtf8BufferSize(topic.Name);
        Span<byte> nameBuffer = stackalloc byte[nameSize];
        InteropHelpers.FillUtf8Buffer(topic.Name, nameBuffer);
        var namePtr = nameBuffer.IsEmpty ? null : (byte*)Unsafe.AsPointer(ref nameBuffer[0]);

        var allocator = RclAllocator.Default.Object;
        var accessor = RosEnvironment.IsSupported(RosEnvironment.Iron)
            ? IronTopicEndPointDataAccessor.Instance
            : TopicEndPointDataAccessor.Instance;

        rmw_topic_endpoint_info_array_t endpoints;
        RclException.ThrowIfNonSuccess(
            rcl_get_publishers_info_by_topic(
                _node.Handle.Object,
                &allocator.Value,
                namePtr,
                disableTopicNameDemangling,
                &endpoints));

        using (var items = SpanOwner<TopicEndPointDataRef>.Allocate((int)endpoints.size.Value))
        {
            try
            {
                for (var i = 0; i < (int)endpoints.size.Value; i++)
                {
                    items.Span[i] = new(&endpoints.info_array[i], accessor);
                }
                topic.UpdatePublishers(this, items.Span, _nodes);
            }
            finally
            {
                rmw_topic_endpoint_info_array_fini(&endpoints, &allocator.Value);
            }
        }

        RclException.ThrowIfNonSuccess(
            rcl_get_subscriptions_info_by_topic(
                _node.Handle.Object,
                &allocator.Value,
                namePtr,
                disableTopicNameDemangling,
                &endpoints));

        using (var items = SpanOwner<TopicEndPointDataRef>.Allocate((int)endpoints.size.Value))
        {
            try
            {
                for (var i = 0; i < (int)endpoints.size.Value; i++)
                {
                    items.Span[i] = new(&endpoints.info_array[i], accessor);
                }
                topic.UpdateSubscribers(this, items.Span, _nodes);
            }
            finally
            {
                rmw_topic_endpoint_info_array_fini(&endpoints, &allocator.Value);
            }
        }
    }

    private unsafe void CopyNameAndTypes(rcl_names_and_types_t* src, Span<NameWithType> dest)
    {
        try
        {
            for (var i = 0; i < (int)src->names.size.Value; i++)
            {
                var name = StringMarshal.CreatePooledString((byte*)src->names.data[i])!;
                for (var j = 0; j < (int)src->types[i].size.Value; j++)
                {
                    var type = StringMarshal
                        .CreatePooledString((byte*)src->types[i].data[j])!;
                    dest[i] = new(name, type);
                }
            }
        }
        finally
        {
            rcl_names_and_types_fini(src);
        }
    }

    private unsafe class TopicEndPointDataAccessor : ITopicEndPointDataAccessor
    {
        public static readonly TopicEndPointDataAccessor Instance = new();

        public GraphId GetGraphId(void* data)
        {
            var item = (rmw_topic_endpoint_info_t*)data;
            return new(item->GetGidSpan());
        }

        public string GetType(void* data)
        {
            var item = (rmw_topic_endpoint_info_t*)data;
            return StringMarshal.CreatePooledString(item->topic_type)!;
        }

        public NodeName GetNodeName(void* data)
        {
            var item = (rmw_topic_endpoint_info_t*)data;
            return new(
                StringMarshal.CreatePooledString(item->node_name)!,
                StringMarshal.CreatePooledString(item->node_namespace)!);
        }

        public QosProfile GetQosProfile(void* data)
        {
            var item = (rmw_topic_endpoint_info_t*)data;
            return QosProfile.Create(in item->qos_profile);
        }
    }

    private unsafe class IronTopicEndPointDataAccessor : ITopicEndPointDataAccessor
    {
        public static readonly TopicEndPointDataAccessor Instance = new();

        public GraphId GetGraphId(void* data)
        {
            var item = (RclIron.rmw_topic_endpoint_info_t*)data;
            return new(item->GetGidSpan());
        }

        public string GetType(void* data)
        {
            var item = (RclIron.rmw_topic_endpoint_info_t*)data;
            return StringMarshal.CreatePooledString(item->topic_type)!;
        }

        public NodeName GetNodeName(void* data)
        {
            var item = (RclIron.rmw_topic_endpoint_info_t*)data;
            return new(
                StringMarshal.CreatePooledString(item->node_name)!,
                StringMarshal.CreatePooledString(item->node_namespace)!);
        }

        public QosProfile GetQosProfile(void* data)
        {
            var item = (RclIron.rmw_topic_endpoint_info_t*)data;
            return QosProfile.Create(in item->qos_profile);
        }
    }
}