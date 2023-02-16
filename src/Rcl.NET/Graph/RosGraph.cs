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
/// <p>
/// All public properities and methods of a <see cref="RosGraph"/> object are thread-safe, which
/// means querying the state of the graph and monitoring graph events concurrently from many threads
/// are allowed.
/// </p>
/// <p>
/// Please note that 'thread-safe' means that any form of access to the
/// <see cref="RosGraph"/> object will never corrupt its internal state.
/// Though if a user tries to query the graph from another thread while
/// <see cref="RosGraph"/> is still building the graph, inconsistent results might be returned.
/// </p>
/// <p>
/// For retrieving consistent results, you can access the <see cref="RosGraph"/> object from
/// the <see cref="RclContext"/> which the <see cref="RosGraph"/> object belongs (e.g. by yielding
/// to the <see cref="RclContext"/> using <c><see langword="await"/> graph.Owner.Context.Yield()</c>).
/// </p>
/// </remarks>
public partial class RosGraph : IGraphBuilder, IObservable<RosGraphEvent>
{
    private long _subscriberId;
    private readonly RclNodeImpl _node;

    private readonly ConcurrentDictionary<long, IObserver<RosGraphEvent>> _observers = new();

    private readonly ConcurrentDictionary<NodeName, RosNode> _nodes = new();
    private readonly ConcurrentDictionary<string, RosTopic> _topics = new();
    private readonly ConcurrentDictionary<string, RosService> _services = new();

    private readonly List<RosTopicEndPoint>
        _totalPublications = new(), _totalSubscriptions = new();
    private readonly List<RosServiceEndPoint>
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

    internal RosGraph(RclNodeImpl node)
    {
        _node = node;
        _cleanupTargets = new IList[]
        {
            _newNodes,_removedNodes,

            _totalPublications, _totalSubscriptions,
            _totalServers, _totalClients,

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

        Nodes = ReadOnlyCollection.Wrap(_nodes.Values);
        Topics = ReadOnlyCollection.Wrap(_topics.Values);
        Services = ReadOnlyCollection.Wrap(_services.Values);
        Actions = ReadOnlyCollection.Wrap(_actions.Values);
    }

    public IRclNode Owner => _node;

    /// <summary>
    /// Returns nodes currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosNode> Nodes { get; }

    /// <summary>
    /// Returns topics currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosTopic> Topics { get; }

    /// <summary>
    /// Returns services currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosService> Services { get; }

    /// <summary>
    /// Returns actions currently available in the ROS graph.
    /// </summary>
    public IReadOnlyCollection<RosAction> Actions { get; }

    internal unsafe void Build()
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
        foreach (var g in _totalPublications.GroupBy(x => x.Node))
        {
            g.Key.ResetPublishers(g);
        }
        foreach (var g in _totalSubscriptions.GroupBy(x => x.Node))
        {
            g.Key.ResetSubscribers(g);
        }

        foreach(var svc in _services.Values)
		{
			svc.ResetClients(_totalClients.Where(x => x.Service == svc));
			svc.ResetServers(_totalServers.Where(x => x.Service == svc));
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

                if (!_nodes.TryGetValue(name, out var node))
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

        using var items = SpanOwner<NameWithType>.Allocate((int)nts.Value.names.size.Value);
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
                    s.UpdatePublishers(this, ReadOnlySpan<TopicEndPointData>.Empty, _nodes);
                    s.UpdateSubscribers(this, ReadOnlySpan<TopicEndPointData>.Empty, _nodes);
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
        if (ret.Value == (int)rcl_ret.RCL_RET_NODE_NAME_NON_EXISTENT ||
            ret.Value == (int)rcl_ret.RCL_RET_OK)
        {
            using var items = SpanOwner<NameWithType>.Allocate((int)nts.Value.names.size.Value);
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
        if (ret.Value == (int)rcl_ret.RCL_RET_NODE_NAME_NON_EXISTENT ||
            ret.Value == (int)rcl_ret.RCL_RET_OK)
        {
            using var items = SpanOwner<NameWithType>.Allocate((int)nts.Value.names.size.Value);
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
        rcl_topic_endpoint_info_array_t endpoints;

        RclException.ThrowIfNonSuccess(
            rcl_get_publishers_info_by_topic(
                _node.Handle.Object,
                &allocator.Value,
                namePtr,
                disableTopicNameDemangling,
                &endpoints));

        using (var items = SpanOwner<TopicEndPointData>.Allocate((int)endpoints.Value.size.Value))
        {
            CopyTopicEndpoints(&endpoints, items.Span, &allocator.Value);
            topic.UpdatePublishers(this, items.Span, _nodes);
        }

        RclException.ThrowIfNonSuccess(
            rcl_get_subscriptions_info_by_topic(
                _node.Handle.Object,
                &allocator.Value,
                namePtr,
                disableTopicNameDemangling,
                &endpoints));

        using (var items = SpanOwner<TopicEndPointData>.Allocate((int)endpoints.Value.size.Value))
        {
            CopyTopicEndpoints(&endpoints, items.Span, &allocator.Value);
            topic.UpdateSubscribers(this, items.Span, _nodes);
        }
    }

    private unsafe void CopyTopicEndpoints(
        rcl_topic_endpoint_info_array_t* src, Span<TopicEndPointData> dest, rcutils_allocator_t* allocator)
    {
        try
        {
            for (var i = 0; i < (int)src->Value.size.Value; i++)
            {
                ref var item = ref src->Value.info_array[i];
                dest[i] = new(
                    new(src->Value.info_array[i].endpoint_gid),
                    StringMarshal.CreatePooledString(item.topic_type)!,
                    new(
                        StringMarshal.CreatePooledString(item.node_name)!,
                        StringMarshal.CreatePooledString(item.node_namespace)!
                    ),
                    QosProfile.Create(in item.qos_profile)
                );
            }
        }
        finally
        {
            rmw_topic_endpoint_info_array_fini(&src->Value, allocator);
        }
    }

    private unsafe void CopyNameAndTypes(rcl_names_and_types_t* src, Span<NameWithType> dest)
    {
        try
        {
            for (var i = 0; i < (int)src->Value.names.size.Value; i++)
            {
                var name = StringMarshal.CreatePooledString((byte*)src->Value.names.data[i])!;
                for (var j = 0; j < (int)src->Value.types[i].size.Value; j++)
                {
                    var type = StringMarshal
                        .CreatePooledString((byte*)src->Value.types[i].data[j])!;
                    dest[i] = new(name, type);
                }
            }
        }
        finally
        {
            rcl_names_and_types_fini(src);
        }
    }
}