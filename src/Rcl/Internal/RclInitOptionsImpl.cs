using Rcl.SafeHandles;

namespace Rcl.Internal;

unsafe class RclInitOptionsImpl : IDisposable
{
    private readonly SafeInitOptionsHandle _handle;

    public RclInitOptionsImpl(RclAllocator? allocator = null)
    {
        _handle = new(allocator);
    }

    public SafeInitOptionsHandle Handle => _handle;

    public nuint DomainId
    {
        get
        {
            size_t ret = default;
            rcl_init_options_get_domain_id(_handle.Object, &ret);
            return ret;
        }
        set
        {
            rcl_init_options_set_domain_id(_handle.Object, value);
        }
    }

    public void Dispose()
    {
        _handle.Dispose();
    }
}