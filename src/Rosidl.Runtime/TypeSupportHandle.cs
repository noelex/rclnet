using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime;

/// <summary>
/// Represents the type of a <see cref="TypeSupportHandle"/>.
/// </summary>
public enum HandleType
{
    /// <summary>
    /// The <see cref="TypeSupportHandle"/> descibes a message interface.
    /// </summary>
    Message,

    /// <summary>
    /// The <see cref="TypeSupportHandle"/> descibes a service interface.
    /// </summary>
    Service,

    /// <summary>
    /// The <see cref="TypeSupportHandle"/> descibes an action interface.
    /// </summary>
    Action
}

/// <summary>
/// A typed wrapper of native type support handle.
/// </summary>
public readonly struct TypeSupportHandle
{
    private readonly nint _handle;
    private readonly HandleType _handleType;

    /// <summary>
    /// Creates a new <see cref="TypeSupportHandle"/>.
    /// </summary>
    /// <param name="handle">The underlying type support handle.</param>
    /// <param name="type">The type of the handle.</param>
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

    /// <summary>
    /// Extract the message type support handle carried by current <see cref="TypeSupportHandle"/>.
    /// </summary>
    /// <returns>A pointer to the native <see cref="MessageTypeSupport"/> handle.</returns>
    public unsafe MessageTypeSupport* GetMessageTypeSupport()
        => (MessageTypeSupport*)ExtractHandle(HandleType.Message);

    /// <summary>
    /// Extract the service type support handle carried by current <see cref="TypeSupportHandle"/>.
    /// </summary>
    /// <returns>A pointer to the native <see cref="ServiceTypeSupport"/> handle.</returns>
    public unsafe ServiceTypeSupport* GetServiceTypeSupport()
        => (ServiceTypeSupport*)ExtractHandle(HandleType.Service);

    /// <summary>
    /// Extract the action type support handle carried by current <see cref="TypeSupportHandle"/>.
    /// </summary>
    /// <returns>A pointer to the native <see cref="ActionTypeSupport"/> handle.</returns>
    public unsafe ActionTypeSupport* GetActionTypeSupport()
        => (ActionTypeSupport*)ExtractHandle(HandleType.Action);

}
