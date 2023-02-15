using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime;

public enum HandleType
{
    Message,
    Service,
    Action
}

public readonly struct TypeSupportHandle
{
    private readonly nint _handle;
    private readonly HandleType _handleType;

    public TypeSupportHandle(nint handle, HandleType type)
    {
        _handle = handle;
        _handleType = type;
    }

    private unsafe void* ExtractHandle(HandleType type)
    {
        if (_handleType != type)
        {
            throw new RosidlException($"Current {nameof(TypeSupportHandle)} is not a {type} handle.");
        }

        return _handle.ToPointer();
    }

    public unsafe MessageTypeSupport* GetMessageTypeSupport()
        => (MessageTypeSupport*)ExtractHandle(HandleType.Message);

    public unsafe ServiceTypeSupport* GetServiceTypeSupport()
        => (ServiceTypeSupport*)ExtractHandle(HandleType.Service);

    public unsafe ActionTypeSupport* GetActionTypeSupport()
        => (ActionTypeSupport*)ExtractHandle(HandleType.Action);

}
