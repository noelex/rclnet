using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime.Interop;

[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct MessageMember_Foxy
{
    public readonly sbyte* Name;
    public readonly FieldType TypeId;
    public readonly nint StringUpperBound;
    public readonly MessageTypeSupport* Members;
    public readonly bool IsArray;
    public readonly nint ArraySize;
    public readonly bool IsUpperBound;
    public readonly int Offset;
    public readonly void* DefaultValue;

    /// <summary>
    /// size_t (* size_function)(const void *);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint> SizeFunction;

    /// <summary>
    /// const void * (*get_const_function)(const void *, size_t index);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, void*> GetConstFunction;

    /// <summary>
    /// void * (*get_function)(void *, size_t index);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, void*> GetFunction;

    /// <summary>
    /// bool (* resize_function)(void *, size_t size);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, bool> ResizeFunction;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct MessageMember_Humble
{
    public readonly sbyte* Name;
    public readonly FieldType TypeId;
    public readonly nint StringUpperBound;
    public readonly MessageTypeSupport* Members;
    public readonly bool IsArray;
    public readonly nint ArraySize;
    public readonly bool IsUpperBound;
    public readonly int Offset;
    public readonly void* DefaultValue;

    /// <summary>
    /// size_t (* size_function)(const void *);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint> SizeFunction;

    /// <summary>
    /// const void * (*get_const_function)(const void *, size_t index);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, void*> GetConstFunction;

    /// <summary>
    /// void * (*get_function)(void *, size_t index);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, void*> GetFunction;

    /// <summary>
    /// void (* fetch_function)(const void *, size_t index, void *);
    /// Requires humble and above.
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, void*, void> FetchFunction;

    /// <summary>
    /// void (* assign_function)(void *, size_t index, const void *);
    /// Requires humble and above.
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint,void*, void> AssignFunction;

    /// <summary>
    /// bool (* resize_function)(void *, size_t size);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, bool> ResizeFunction;
}

[StructLayout(LayoutKind.Sequential)]
public unsafe readonly struct MessageMembers
{
    public readonly sbyte* MessageNamespace;
    public readonly sbyte* MessageName;
    public readonly int MemberCount;
    public readonly nint SizeOf;
    public readonly void* Members;

    /// <summary>
    /// void (* init_function)(void *, enum rosidl_runtime_c__message_initialization);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, MessageInitialization, void> InitFunction;

    /// <summary>
    /// void (* fini_function)(void *);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, void> FiniFunction;
}