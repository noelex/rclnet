using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeArgumentsHandle : RclObjectHandle<rcl_arguments_t>
{
    public SafeArgumentsHandle(string[] args)
    {
        try
        {
            var allocator = RclAllocator.Default;
            *DangerousObject = rcl_get_zero_initialized_arguments();

            int argc = args.Length;
            if (argc > 0)
            {
                var bufferSize = InteropHelpers.GetUtf8BufferSize(args);
                Span<int> argOffsets = stackalloc int[argc];
                Span<byte> argBuffer = stackalloc byte[bufferSize];
                var argv = stackalloc byte*[argc];
                InteropHelpers.FillUtf8Buffer(args, argBuffer, argv);

                RclException.ThrowIfNonSuccess(rcl_parse_arguments(argc, argv, allocator.Object, DangerousObject));
            }
            else
            {
                RclException.ThrowIfNonSuccess(rcl_parse_arguments(0, null, allocator.Object, DangerousObject));
            }
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    private SafeArgumentsHandle(rcl_arguments_t* pointer, RclObjectHandle owner)
        : base((IntPtr)pointer, owner) { }

    internal static SafeArgumentsHandle Borrow(SafeContextHandle context)
    {
        using var lease = context.Acquire();
        return new(&lease.Object->global_arguments, context);
    }

    internal static SafeArgumentsHandle Borrow(SafeNodeHandle node)
    {
        using var lease = node.Acquire();
        var options = rcl_node_get_options(lease.Object);
        var arguments = RosEnvironment.IsFoxy
            ? &((RclFoxy.rcl_node_options_t*)options)->arguments
            : &((RclHumble.rcl_node_options_t*)options)->arguments;
        return new(arguments, node);
    }

    protected override bool ReleaseHandleCore(rcl_arguments_t* ptr)
    {
        return CheckReleaseResult(rcl_arguments_fini(ptr), nameof(rcl_arguments_fini));
    }
}
