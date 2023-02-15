namespace Rcl.SafeHandles;

unsafe class SafeInitOptionsHandle : RclObjectHandle<rcl_init_options_t>
{
    public SafeInitOptionsHandle(RclAllocator? allocator = null)
    {
        *Object = rcl_get_zero_initialized_init_options();
        rcl_init_options_init(Object, (allocator ?? RclAllocator.Default).Object);
    }

    protected override void ReleaseHandleCore(rcl_init_options_t* ptr)
    {
        rcl_init_options_fini(ptr);
    }
}