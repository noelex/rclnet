using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace Rcl.SafeHandles;

unsafe class SafeSubscriptionHandle : RclObjectHandle<rcl_subscription_t>
{
    private readonly SafeNodeHandle _node;

    public SafeSubscriptionHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string topicName, SubscriptionOptions options)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_subscription();

        var nameSize = InteropHelpers.GetUtf8BufferSize(topicName);
        byte* name = stackalloc byte[nameSize];
        InteropHelpers.FillUtf8Buffer(topicName, new(name, nameSize));

        if (RosEnvironment.IsFoxy)
        {
            InitFoxy(name, typeSupportHandle, options);
        }
        else
        {
            InitHumbleOrLater(name, typeSupportHandle, options);
        }
    }

    private void InitFoxy(byte* name, TypeSupportHandle typeSupport, SubscriptionOptions options)
    {
        // This is backward compatible as long as we don't access
        // rmw_subscription_options.require_unique_network_flow_endpoints and
        // rmw_subscription_options.content_filter_options
        var opts = RclHumble.rcl_subscription_get_default_options();
        opts.qos = options.Qos.ToRmwQosProfile();
        opts.rmw_subscription_options.ignore_local_publications = options.IgnoreLocalPublications;

        RclException.ThrowIfNonSuccess(
                rcl_subscription_init(
                    Object,
                    _node.Object,
                    typeSupport.GetMessageTypeSupport(),
                    name,
                    &opts));

    }

    private void InitHumbleOrLater(byte* name, TypeSupportHandle typeSupport, SubscriptionOptions options)
    {
        var opts = RclHumble.rcl_subscription_get_default_options();

        opts.qos = options.Qos.ToRmwQosProfile();
        opts.rmw_subscription_options.ignore_local_publications = options.IgnoreLocalPublications;

        if (options.ContentFilter != null)
        {
            var expSize = InteropHelpers.GetUtf8BufferSize(options.ContentFilter.Expression);
            byte* expBuffer = stackalloc byte[expSize];
            InteropHelpers.FillUtf8Buffer(options.ContentFilter.Expression, new(expBuffer, expSize));

            var argc = options.ContentFilter.Arguments.Length;
            rcl_ret_t ret;
            if (argc > 0)
            {
                var bufferSize = InteropHelpers.GetUtf8BufferSize(options.ContentFilter.Arguments);
                Span<byte> argBuffer = stackalloc byte[bufferSize];
                byte** argv = stackalloc byte*[argc];
                InteropHelpers.FillUtf8Buffer(options.ContentFilter.Arguments, argBuffer, argv);
                ret = RclHumble.rcl_subscription_options_set_content_filter_options(
                    expBuffer,
                    (uint)argc,
                    argv,
                    &opts);
            }
            else
            {
                ret = RclHumble.rcl_subscription_options_set_content_filter_options(
                    expBuffer,
                    0,
                    null,
                    &opts);
            }

            RclException.ThrowIfNonSuccess(ret);
        }

        RclException.ThrowIfNonSuccess(
                rcl_subscription_init(
                    Object,
                    _node.Object,
                    typeSupport.GetMessageTypeSupport(),
                    name,
                    &opts));
    }

    protected override void ReleaseHandleCore(rcl_subscription_t* ptr)
    {
        rcl_subscription_fini(ptr, _node.Object);
    }
}