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
    enum UpdateOp { Add = 1, Remove = 2 }

    private long _subscriberId;
    private readonly RclNodeImpl _node;

    private readonly ConcurrentDictionary<long, IObserver<RosGraphEvent>> _observers = new();

    private readonly ConcurrentDictionary<string, RosNode> _nodes = new();
    private readonly ConcurrentDictionary<string, RosTopic> _topics = new();
    private readonly ConcurrentDictionary<string, RosService> _services = new();

    private readonly Dictionary<RosService, PoolingList<RosServiceEndPoint>>
        _totalServers = new(), _totalClients = new();

    private readonly Dictionary<RosNode, UpdateOp> _nodeUpdates = new();
    private readonly Dictionary<RosService, UpdateOp> _serviceUpdates = new();
    private readonly Dictionary<RosServiceEndPoint, UpdateOp> _serverUpdates = new(), _clientUpdates = new();
    private readonly Dictionary<RosTopic, UpdateOp> _topicUpdates = new();
    private readonly Dictionary<RosTopicEndPoint, UpdateOp> _publisherUpdates = new(), _subscriberUpdates = new();

    private readonly IDictionary[] _cleanupTargets;
    private readonly Func<NodeName, bool> _nodeFilter;

    private readonly IEnumerator<KeyValuePair<string, RosNode>> _nodesEnumerator;
    private readonly IEnumerator<KeyValuePair<string, RosService>> _servicesEnumerator;
    private readonly IEnumerator<KeyValuePair<string, RosTopic>> _topicsEnumerator;
    private readonly IEnumerator<KeyValuePair<long, IObserver<RosGraphEvent>>> _observersEnumerator;

    internal RosGraph(RclNodeImpl node, Func<NodeName, bool> nodeFilter)
    {
        _node = node;
        _nodesEnumerator = _nodes.GetEnumerator();
        _actionsEnumerator = _actions.GetEnumerator();
        _servicesEnumerator = _services.GetEnumerator();
        _topicsEnumerator = _topics.GetEnumerator();
        _observersEnumerator = _observers.GetEnumerator();

        _cleanupTargets = new IDictionary[]
        {
            _nodeUpdates,
            _serviceUpdates, _serverUpdates, _clientUpdates,
            _topicUpdates, _publisherUpdates, _subscriberUpdates,
            _actionUpdates, _actionServerUpdates, _actionClientUpdates,

            _totalServers, _totalClients,
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
        foreach (var (svc, endpoints) in _totalServers)
        {
            svc.ResetServers(endpoints.AsSpan());
        }
        foreach (var (svc, endpoints) in _totalClients)
        {
            svc.ResetClients(endpoints.AsSpan());
        }

        try
        {
            while (_servicesEnumerator.MoveNext())
            {
                var (k, v) = _servicesEnumerator.Current;
                if ((!_totalServers.ContainsKey(v) && !_totalClients.ContainsKey(v)) || 
                    (v.ClientCount == 0 && v.ServerCount == 0))
                {
                    _services.Remove(k, out _);
                    OnRemove(_serviceUpdates, v);
                }
            }
        }
        finally
        {
            _servicesEnumerator.Reset();
        }
    }

    private static void OnAdd<T>(Dictionary<T, UpdateOp> target, T item) where T : class
        => target[item] = UpdateOp.Add;

    private static void OnRemove<T>(Dictionary<T, UpdateOp> target, T item) where T : class
        => target[item] = UpdateOp.Remove;

    private void Cleanup()
    {
        foreach (var v in _totalServers.Values) v.Dispose();
        foreach (var v in _totalClients.Values) v.Dispose();

        foreach (var list in _cleanupTargets)
        {
            list.Clear();
        }

        _totalActionClients.Clear();
        _totalActionServers.Clear();
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
            using var discoveredNodes = SpanOwner<string>.Allocate((int)names.size.Value);

            for (var i = 0; i < (int)names.size.Value; i++)
            {
                var name = new NodeName(
                    StringMarshal.CreatePooledString((byte*)names.data[i])!,
                    StringMarshal.CreatePooledString((byte*)namespaces.data[i])!);
                var fqn = name.FullyQualifiedName;

                if (_nodeFilter(name) && !_nodes.TryGetValue(fqn, out var node))
                {
                    var enclave = StringMarshal.CreatePooledString((byte*)enclaves.data[i])!;
                    _nodes[fqn] = node = new RosNode(name, enclave);
                    OnAdd(_nodeUpdates, node);
                }

                discoveredNodes.Span[i] = fqn;
            }

            try
            {
                while (_nodesEnumerator.MoveNext())
                {
                    var (k, node) = _nodesEnumerator.Current;
                    if (discoveredNodes.Span.IndexOf(k) < 0)
                    {
                        if (_nodes.Remove(k, out var v))
                        {
                            OnRemove(_nodeUpdates, v);

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
                _nodesEnumerator.Reset();
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
                OnAdd(_topicUpdates, topic);
            }
        }

        try
        {
            while (_topicsEnumerator.MoveNext())
            {
                var (k, v) = _topicsEnumerator.Current;
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
                        OnRemove(_topicUpdates, s);

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
        finally
        {
            _topicsEnumerator.Reset();
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