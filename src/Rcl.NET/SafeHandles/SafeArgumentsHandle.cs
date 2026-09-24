using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeArgumentsHandle : RclObjectHandle<rcl_arguments_t>
{
    public SafeArgumentsHandle(string[] args)
    {
        try
        {
            var allocator = RclAllocator.Default;
            *Object = rcl_get_zero_initialized_arguments();

            int argc = args.Length;
            if (argc > 0)
            {
                var bufferSize = InteropHelpers.GetUtf8BufferSize(args);
                Span<int> argOffsets = stackalloc int[argc];
                Span<byte> argBuffer = stackalloc byte[bufferSize];
                var argv = stackalloc byte*[argc];
                InteropHelpers.FillUtf8Buffer(args, argBuffer, argv);

                RclException.ThrowIfNonSuccess(rcl_parse_arguments(argc, argv, allocator.Object, Object));
            }
            else
            {
                RclException.ThrowIfNonSuccess(rcl_parse_arguments(0, null, allocator.Object, Object));
            }
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    public SafeArgumentsHandle(IntPtr handle) : base(handle)
    {
    }

    protected override bool ReleaseHandleCore(rcl_arguments_t* ptr)
    {
        return CheckReleaseResult(rcl_arguments_fini(ptr), nameof(rcl_arguments_fini));
    }
}
