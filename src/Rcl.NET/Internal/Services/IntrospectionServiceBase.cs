using Rcl.Interop;
using Rcl.Introspection;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Internal.Services;

internal abstract class IntrospectionServiceBase : RclWaitObject<SafeServiceHandle>, IRclService
{
    private readonly RclNodeImpl _node;
    private readonly ServiceIntrospection _typesupport;

    public unsafe IntrospectionServiceBase(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typesupport,
        ServerOptions options)
        : base(node.Context, new(node.Handle, node.Clock.Impl.Handle, typesupport, serviceName, options.Qos))
    {
        try
        {
            using var lease = Handle.Acquire();
            _node = node;
            _typesupport = new ServiceIntrospection(typesupport);

            Name = StringMarshal.CreatePooledString(rcl_service_get_service_name(lease.Object))!;
        }
        catch
        {
            Handle.Dispose();
            throw;
        }
    }

    protected virtual RosMessageBuffer CreateRequestBuffer()
        => _typesupport.Request.CreateBuffer();

    protected virtual RosMessageBuffer CreateResponseBuffer()
        => _typesupport.Response.CreateBuffer();

    public string Name { get; }

    public unsafe bool IsValid
    {
        get
        {
            using var lease = Handle.Acquire();

            lock (Handle.NativeGate)
            {
                return rcl_service_is_valid(lease.Object);
            }
        }
    }

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        var requestBuffer = CreateRequestBuffer();

        rcl_ret_t result;

        try
        {
            using var lease = Handle.Acquire();

            lock (Handle.NativeGate)
            {
                result = rcl_take_request_with_info(lease.Object, &header, requestBuffer.Data.ToPointer());
            }
        }
        catch
        {
            OnTakeRequestFailed(requestBuffer);
            throw;
        }

        if (result == rcl_ret_t.RCL_RET_OK)
        {
            RosMessageBuffer responseBuffer;

            try
            {
                responseBuffer = CreateResponseBuffer();
            }
            catch
            {
                requestBuffer.Dispose();
                throw;
            }

            DispatchRequest(requestBuffer, responseBuffer, header.request_id);
        }
        else
        {
            OnTakeRequestFailed(requestBuffer);
        }
    }

    protected virtual void OnTakeRequestFailed(RosMessageBuffer requestBuffer)
    {
        requestBuffer.Dispose();
    }

    protected abstract unsafe void DispatchRequest(RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t id);

    protected unsafe rcl_ret_t SendResponse(rmw_request_id_t id, IntPtr data)
    {
        using var lease = Handle.Acquire();

        lock (Handle.NativeGate)
        {
            return rcl_send_response(lease.Object, &id, data.ToPointer());
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
                var ret = RclIron.rcl_service_configure_service_introspection(
                    lease.Object,
                    _node.Handle.DangerousObject,
                    _node.Clock.Impl.Handle.DangerousObject,
                    _typesupport.TypeSupportHandle,
                    opts,
                    (RclIron.rcl_service_introspection_state_t)state);

                RclException.ThrowIfNonSuccess(ret);
            }
        }
    }
}
