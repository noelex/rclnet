using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeContextHandle : RclObjectHandle<rcl_context_t>
{
    public SafeContextHandle(string[] args)
    {
        *Object = rcl_get_zero_initialized_context();
        var opts = rcl_get_zero_initialized_init_options();

        try
        {
            RclException.ThrowIfNonSuccess(rcl_init_options_init(&opts, RclAllocator.Default.Object));

            int argc = args.Length;
            if (argc > 0)
            {
                var bufferSize = InteropHelpers.GetUtf8BufferSize(args);
                Span<byte> argBuffer = stackalloc byte[bufferSize];
                var argv = stackalloc byte*[argc];
                InteropHelpers.FillUtf8Buffer(args, argBuffer, argv);
                RclException.ThrowIfNonSuccess(rcl_init(argc, argv, &opts, Object));
            }
            else
            {
                RclException.ThrowIfNonSuccess(rcl_init(0, null, &opts, Object));
            }
        }
        catch
        {
            Dispose();
            throw;
        }
        finally
        {
            rcl_init_options_fini(&opts);
        }
    }

    protected override void ReleaseHandleCore(rcl_context_t* ptr)
    {
        rcl_shutdown(ptr);
    }
}