using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeNodeHandle : RclObjectHandle<rcl_node_t>
{
    public SafeNodeHandle(SafeContextHandle context, string name, string @namespace, NodeOptions options)
    {
        try
        {
            lock (context.LifecycleGate)
            {
                SetDependencies(context);
                *DangerousObject = rcl_get_zero_initialized_node();
                var nameSize = InteropHelpers.GetUtf8BufferSize(name);
                var nsSize = InteropHelpers.GetUtf8BufferSize(@namespace);
                Span<byte> nameBuffer = stackalloc byte[nameSize];
                Span<byte> nsBuffer = stackalloc byte[nsSize];
                InteropHelpers.FillUtf8Buffer(name, nameBuffer);
                InteropHelpers.FillUtf8Buffer(@namespace, nsBuffer);

                fixed (byte* namePtr = nameBuffer)
                fixed (byte* nsPtr = nsBuffer)
                {
                    switch (RosEnvironment.Distribution)
                    {
                        case RosEnvironment.Foxy: InitFoxy(namePtr, nsPtr, context, options); break;
                        case RosEnvironment.Humble:
                        case RosEnvironment.Iron:
                        case RosEnvironment.Jazzy:
                        case RosEnvironment.Kilted:
                        case RosEnvironment.Lyrical:
                            InitHumbleOrLater(namePtr, nsPtr, context, options);
                            break;
                        default: throw new NotImplementedException();
                    }
                }
                MarkInitialized();
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    private void InitFoxy(byte* namePtr, byte* nsPtr, SafeContextHandle context, NodeOptions options)
    {
        var opts = RclFoxy.rcl_node_get_default_options();
        if (options.DomaindId != null)
        {
            opts.domain_id = (size_t)options.DomaindId;
        }

        using var arguments = new SafeArgumentsHandle(options.Arguments);
        opts.arguments = *arguments.DangerousObject;

        opts.use_global_arguments = options.UseGlobalArguments;
        opts.enable_rosout = options.EnableRosOut;

        lock (SafeContextHandle.LoggingGate)
            RclException.ThrowIfNonSuccess(
                rcl_node_init(DangerousObject, namePtr, nsPtr, context.DangerousObject, &opts));
    }

    private void InitHumbleOrLater(byte* namePtr, byte* nsPtr, SafeContextHandle context, NodeOptions options)
    {
        var opts = RclHumble.rcl_node_get_default_options();

        using var arguments = new SafeArgumentsHandle(options.Arguments);
        opts.arguments = *arguments.DangerousObject;

        opts.use_global_arguments = options.UseGlobalArguments;
        opts.enable_rosout = options.EnableRosOut;
        opts.rosout_qos = options.RosOutQos.ToRmwQosProfile();

        lock (SafeContextHandle.LoggingGate)
            RclException.ThrowIfNonSuccess(
                rcl_node_init(DangerousObject, namePtr, nsPtr, context.DangerousObject, &opts));
    }

    protected override bool ReleaseHandleCore(rcl_node_t* ptr)
    {
        lock (SafeContextHandle.LoggingGate)
            return CheckReleaseResult(rcl_node_fini(ptr), nameof(rcl_node_fini));
    }
}
