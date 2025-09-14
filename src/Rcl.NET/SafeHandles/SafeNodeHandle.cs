using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeNodeHandle : RclObjectHandle<rcl_node_t>
{
    // Creating / disposing nodes with /rosout logging enabled requires access
    // to a static logger map, which is not thread-safe application wide.
    // Thus a global lock is required here.
    private static SpinLock s_nodeCreationLock = new();

    public SafeNodeHandle(SafeContextHandle context, string name, string @namespace, NodeOptions options)
    {
        *Object = rcl_get_zero_initialized_node();

        try
        {
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
                        InitHumbleOrLater(namePtr, nsPtr, context, options);
                        break;
                    default: throw new NotImplementedException();
                }
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

        ParseArguments(options.Arguments, &opts.arguments);

        opts.use_global_arguments = options.UseGlobalArguments;
        opts.enable_rosout = options.EnableRosOut;

        using (ScopedLock.Lock(ref s_nodeCreationLock))
            RclException.ThrowIfNonSuccess(
                rcl_node_init(Object, namePtr, nsPtr, context.Object, &opts));
    }

    private void InitHumbleOrLater(byte* namePtr, byte* nsPtr, SafeContextHandle context, NodeOptions options)
    {
        var opts = RclHumble.rcl_node_get_default_options();

        ParseArguments(options.Arguments, &opts.arguments);

        opts.use_global_arguments = options.UseGlobalArguments;
        opts.enable_rosout = options.EnableRosOut;
        opts.rosout_qos = options.RosOutQos.ToRmwQosProfile();

        using (ScopedLock.Lock(ref s_nodeCreationLock))
            RclException.ThrowIfNonSuccess(
                rcl_node_init(Object, namePtr, nsPtr, context.Object, &opts));
    }

    private void ParseArguments(string[] args, rcl_arguments_t* result)
    {
        var allocator = RclAllocator.Default;

        int argc = args.Length;
        if (argc > 0)
        {
            var bufferSize = InteropHelpers.GetUtf8BufferSize(args);
            Span<int> argOffsets = stackalloc int[argc];
            Span<byte> argBuffer = stackalloc byte[bufferSize];
            var argv = stackalloc byte*[argc];
            InteropHelpers.FillUtf8Buffer(args, argBuffer, argv);

            rcl_parse_arguments(argc, argv, allocator.Object, result);
        }
        else
        {
            rcl_parse_arguments(0, null, allocator.Object, result);
        }
    }

    protected override void ReleaseHandleCore(rcl_node_t* ptr)
    {
        using (ScopedLock.Lock(ref s_nodeCreationLock)) rcl_node_fini(ptr);
    }
}