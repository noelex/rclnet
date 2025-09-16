using System.Runtime.InteropServices;

namespace Rosidl.Runtime.Interop;

/// <summary>
/// Contains rosidl message type support data
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct MessageTypeSupport
{
    /// <summary>
    /// String identifier for the type_support.
    /// </summary>
    public readonly sbyte* TypeSupportIdentifier;

    /// <summary>
    /// Pointer to the message type support library
    /// </summary>
    public readonly void* Data;

    /// <summary>
    /// Pointer to the message type support handler function
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<MessageTypeSupport*, sbyte*, MessageTypeSupport*> Handler;

}

/// <summary>
/// Contains rosidl message type support data
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct MessageTypeSupport_Jazzy
{
    /// <summary>
    /// String identifier for the type_support.
    /// </summary>
    public readonly sbyte* TypeSupportIdentifier;

    /// <summary>
    /// Pointer to the message type support library
    /// </summary>
    public readonly void* Data;

    /// <summary>
    /// Pointer to the message type support handler function
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<MessageTypeSupport_Jazzy*, sbyte*, MessageTypeSupport_Jazzy*> Handler;

    // Keep memory layout consistent
    internal readonly IntPtr get_type_hash_func;
    internal readonly IntPtr get_type_description_func;
    internal readonly IntPtr get_type_description_sources_func;
}

/// <summary>
/// Contains rosidl service type support data
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct ServiceTypeSupport
{
    /// <summary>
    /// String identifier for the type_support.
    /// </summary>
    public readonly sbyte* TypeSupportIdentifier;

    /// <summary>
    /// Pointer to the message type support library
    /// </summary>
    public readonly void* Data;

    /// <summary>
    /// Pointer to the message type support handler function
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<ServiceTypeSupport*, sbyte*, ServiceTypeSupport*> Handler;
}

/// <summary>
/// Contains rosidl service type support data
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct ServiceTypeSupport_Jazzy
{
    /// <summary>
    /// String identifier for the type_support.
    /// </summary>
    public readonly sbyte* TypeSupportIdentifier;

    /// <summary>
    /// Pointer to the message type support library
    /// </summary>
    public readonly void* Data;

    /// <summary>
    /// Pointer to the message type support handler function
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<ServiceTypeSupport_Jazzy*, sbyte*, ServiceTypeSupport_Jazzy*> Handler;


    // Keep memory layout consistent
    internal readonly IntPtr request_typesupport;
    internal readonly IntPtr response_typesupport;
    internal readonly IntPtr event_typesupport;
    internal readonly IntPtr event_message_create_handle_function;
    internal readonly IntPtr event_message_destroy_handle_function;
    internal readonly IntPtr get_type_hash_func;
    internal readonly IntPtr get_type_description_func;
    internal readonly IntPtr get_type_description_sources_func;
}

/// <summary>
/// Contains rosidl action type support data
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct ActionTypeSupport
{
    /// <summary>
    /// The native type support handle of the goal service.
    /// </summary>
    public readonly ServiceTypeSupport* GoalService;

    /// <summary>
    /// The native type support handle of the result service.
    /// </summary>
    public readonly ServiceTypeSupport* ResultService;

    /// <summary>
    /// The native type support handle of the cancel service.
    /// </summary>
    public readonly ServiceTypeSupport* CancelService;

    /// <summary>
    /// The native type support handle of the feedback message.
    /// </summary>
    public readonly MessageTypeSupport* FeedbackMessage;

    /// <summary>
    /// The native type support handle of the status message.
    /// </summary>
    public readonly MessageTypeSupport* StatusMessage;
}

/// <summary>
/// Contains rosidl action type support data
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct ActionTypeSupport_Jazzy
{
    /// <summary>
    /// The native type support handle of the goal service.
    /// </summary>
    public readonly ServiceTypeSupport* GoalService;

    /// <summary>
    /// The native type support handle of the result service.
    /// </summary>
    public readonly ServiceTypeSupport* ResultService;

    /// <summary>
    /// The native type support handle of the cancel service.
    /// </summary>
    public readonly ServiceTypeSupport* CancelService;

    /// <summary>
    /// The native type support handle of the feedback message.
    /// </summary>
    public readonly MessageTypeSupport* FeedbackMessage;

    /// <summary>
    /// The native type support handle of the status message.
    /// </summary>
    public readonly MessageTypeSupport* StatusMessage;

    // Keep memory layout consistent
    internal readonly IntPtr get_type_hash_func;
    internal readonly IntPtr get_type_description_func;
    internal readonly IntPtr get_type_description_sources_func;
}