namespace Rcl.Internal;

unsafe class RclAllocator
{
    internal readonly rcl_allocator_t Object;

    internal RclAllocator(rcl_allocator_t impl)
    {
        Object = impl;
    }

    public void Deallocate(void* ptr)
    {
        Object.Value.deallocate(ptr, Object.Value.state);
    }

    public static RclAllocator Default { get; } = new RclAllocator(rcutils_get_default_allocator());
}