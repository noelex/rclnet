using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Internal.Clients;

internal abstract class RclClientBase : RclWaitObject<SafeClientHandle>
{
    private readonly TypeSupportHandle _typesupport;
    private readonly RclNodeImpl _node;
    private readonly object _pendingGate = new();
    private readonly Action<PendingOperation<RosMessageBuffer>, Exception> _cancelPending;
    private readonly Dictionary<long, PendingOperation<RosMessageBuffer>> _pendingRequests = new();
    private RclTimeProvider? _timeoutProvider;
    private bool _pendingClosed;

    public unsafe RclClientBase(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typeSupport,
        ClientOptions options)
        : base(node.Context, new(node.Handle, node.Clock.Impl.Handle, typeSupport, serviceName, options.Qos))
    {
        try
        {
            using var lease = Handle.Acquire();
            _node = node;
            _cancelPending = Cancel;
            _typesupport = typeSupport;
            Name = StringMarshal.CreatePooledString(rcl_client_get_service_name(lease.Object))!;

            if (RosEnvironment.IsSupported(RosEnvironment.Iron))
            {
                RclIron.rmw_gid_t gid;
                var handle = rcl_client_get_rmw_handle(lease.Object);
                RclException.ThrowIfNonSuccess(RclIron.rmw_get_gid_for_client(handle, &gid));
                Gid = new(gid.GetGidSpan());
            }
        }
        catch
        {
            Handle.Dispose();
            throw;
        }
    }

    public unsafe bool IsServerAvailable
    {
        get
        {
            using var lease = Handle.Acquire();
            bool available;

            lock (Handle.NativeGate)
            {
                RclException.ThrowIfNonSuccess(
                rcl_service_server_is_available(_node.Handle.DangerousObject, lease.Object, &available));
            }

            return available;
        }
    }

    public unsafe void ConfigureIntrospection(ServiceIntrospectionState state, QosProfile? qos = null)
    {
        RosEnvironment.Require(RosEnvironment.Iron, feature: "Service Introspection");

        var opts = RclIron.rcl_publisher_get_default_options();
        opts.qos = (qos ?? QosProfile.SystemDefault).ToRmwQosProfile();

        // Configuration can create a publisher: admission precedes native-state locks.
        lock (Context.Handle.LifecycleGate)
        {
            using var lease = Handle.Acquire();
            Handle.ThrowIfDescendantClosed();

            lock (Handle.NativeGate)
            {
                var ret = RclIron.rcl_client_configure_service_introspection(
                    lease.Object,
                    _node.Handle.DangerousObject,
                    _node.Clock.Impl.Handle.DangerousObject,
                    _typesupport.GetServiceTypeSupport(),
                    opts,
                    (RclIron.rcl_service_introspection_state_t)state);

                RclException.ThrowIfNonSuccess(ret);
            }
        }
    }

    public Task<bool> TryWaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => _node.Graph.TryWaitForServiceServerAsync(Name!, timeoutMilliseconds, cancellationToken);

    public Task<bool> TryWaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        => _node.Graph.TryWaitForServiceServerAsync(Name!, timeout, cancellationToken);

    public Task WaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => _node.Graph.WaitForServiceServerAsync(Name!, timeoutMilliseconds, cancellationToken);

    public Task WaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        => _node.Graph.WaitForServiceServerAsync(Name!, timeout, cancellationToken);

    public Task WaitForServerAsync(CancellationToken cancellationToken = default)
        => _node.Graph.WaitForServiceServerAsync(Name!, cancellationToken);

    public string Name { get; }

    public GraphId Gid { get; }

    public unsafe bool IsValid
    {
        get
        {
            using var lease = Handle.Acquire();

            lock (Handle.NativeGate)
            {
                return rcl_client_is_valid(lease.Object);
            }
        }
    }

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        var responseBuffer = CreateResponseBuffer();
        var keepBuffer = false;

        try
        {
            rcl_ret_t result;

            using (var lease = Handle.Acquire())
            {
                lock (Handle.NativeGate)
                {
                    result = rcl_take_response_with_info(lease.Object, &header, responseBuffer.Data.ToPointer());
                }
            }

            if (result == rcl_ret_t.RCL_RET_OK)
            {
                PendingOperation<RosMessageBuffer>? pending;

                lock (_pendingGate)
                {
                    _pendingRequests.Remove(header.request_id.sequence_number, out pending);
                }

                if (pending != null)
                {
                    try
                    {
                        keepBuffer = pending.Succeed(responseBuffer);
                    }
                    finally
                    {
                        pending.Release();
                    }
                }
            }
        }
        finally
        {
            if (!keepBuffer)
            {
                responseBuffer.Dispose();
            }
        }
    }

    protected abstract RosMessageBuffer CreateResponseBuffer();

    public Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        try
        {
            return InvokeCore(request, timeout, cancellationToken).AsTask();
        }
        catch (OperationCanceledException error)
        {
            return Task.FromCanceled<RosMessageBuffer>(error.CancellationToken);
        }
        catch (Exception error)
        {
            return Task.FromException<RosMessageBuffer>(error);
        }
    }

    protected ValueTask<RosMessageBuffer> InvokeCore(RosMessageBuffer request, TimeSpan timeout, CancellationToken cancellationToken)
    {
        Handle.ThrowIfOperationClosed();
        cancellationToken.ThrowIfCancellationRequested();

        if (timeout != Timeout.InfiniteTimeSpan)
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(timeout, TimeSpan.Zero);
            ArgumentOutOfRangeException.ThrowIfGreaterThan(timeout.TotalMilliseconds, uint.MaxValue - 1, nameof(timeout));
        }

        var pending = PendingOperation<RosMessageBuffer>.Rent(true, _cancelPending);
        bool published = false;

        try
        {
            // The response path removes entries under the same gate after native take.
            // Thus even an immediate response cannot overtake sequence publication.
            pending.SetupCancellation(cancellationToken, timeout,
                timeout == Timeout.InfiniteTimeSpan ? null : GetTimeoutProvider());
            SendAndPublish();
        }
        catch (Exception error)
        {
            if (published)
            {
                Cancel(pending, error);
            }
            else
            {
                pending.Fail(error);
            }
        }
        finally
        {
            pending.FinishSetup();
        }

        return pending.Task;

        unsafe void SendAndPublish()
        {
            using var lease = Handle.Acquire();

            lock (_pendingGate)
            {
                Handle.ThrowIfOperationClosed();
                ObjectDisposedException.ThrowIf(_pendingClosed, this);
                if (pending.IsCompleted)
                {
                    return;
                }

                long sequence;

                lock (Handle.NativeGate)
                {
                    RclException.ThrowIfNonSuccess(rcl_send_request(lease.Object, request.Data.ToPointer(), &sequence));
                }

                pending.Key = sequence;
                _pendingRequests.Add(sequence, pending);
                pending.AddReference();
                published = true;
            }
        }
    }

    private RclTimeProvider GetTimeoutProvider()
    {
        lock (_pendingGate)
        {
            ObjectDisposedException.ThrowIf(_pendingClosed, this);
            // The client owns this provider so node disposal cannot stop request timeouts.
            return _timeoutProvider ??= new RclTimeProvider(Context, _node.Clock);
        }
    }

    private void Cancel(PendingOperation<RosMessageBuffer> pending, Exception error)
    {
        lock (_pendingGate)
        {
            bool removed = false;
            if (_pendingRequests.TryGetValue(pending.Key, out var current) && ReferenceEquals(current, pending))
            {
                _pendingRequests.Remove(pending.Key);
                removed = true;
            }

            // Cancellation can win during setup, before a sequence has been published.
            // Publish the terminal state under the send gate so it cannot be missed.
            try
            {
                pending.Fail(error);
            }
            finally
            {
                if (removed)
                {
                    pending.Release();
                }
            }
        }
    }

    public Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => InvokeAsync(request, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    public Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, CancellationToken cancellationToken = default)
        => InvokeAsync(request, Timeout.InfiniteTimeSpan, cancellationToken);

    protected override void StopPendingOperations()
    {
        PendingOperation<RosMessageBuffer>[] snapshot;

        lock (_pendingGate)
        {
            _pendingClosed = true;
            snapshot = _pendingRequests.Values.ToArray();
            _pendingRequests.Clear();
        }

        foreach (var pending in snapshot)
        {
            try
            {
                pending.Fail(new ObjectDisposedException(GetType().Name), asynchronous: true);
            }
            finally
            {
                pending.Release();
            }
        }
    }

    protected override void OnStopped()
    {
        _timeoutProvider?.Dispose();
    }
}
