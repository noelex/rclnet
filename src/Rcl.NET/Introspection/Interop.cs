using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Introspection;

enum MessageInitialization
{
    /// <summary>
    /// Initialize all fields of the message, either with the default value
    /// (if the field has one), or with an empty value (generally 0 or an
    /// empty string).
    /// </summary>
    All,

    /// <summary>
    /// Skip initialization of all fields of the message.  It is up to the user to
    /// ensure that all fields are initialized before use.
    /// </summary>
    Skip,

    /// <summary>
    /// Initialize all fields of the message to an empty value (generally 0 or an
    /// empty string).
    /// </summary>
    Zero,

    /// <summary>
    /// Initialize all fields of the message that have defaults; all other fields
    /// are left untouched.
    /// </summary>
    DefaultsOnly
}

enum FieldType : byte
{
    Float = 1,
    Double = 2,
    LongDouble = 3,
    Char = 4,
    WChar = 5,
    Boolean = 6,
    Octet = 7,
    UInt8 = 8,
    Int8 = 9,
    UInt16 = 10,
    Int16 = 11,
    UInt32 = 12,
    Int32 = 13,
    UInt64 = 14,
    Int64 = 15,
    String = 16,
    WString = 17,
    Message = 18,

    [Obsolete("Use FieldType.Float instead.")]
    Float32 = 1,

    [Obsolete("Use FieldType.Double instead.")]
    Float64 = 2,

    [Obsolete("Use FieldType.Boolean instead.")]
    Bool = 6,

    [Obsolete("Use FieldType.UInt8 instead.")]
    Byte = 7
}


[StructLayout(LayoutKind.Sequential)]
unsafe readonly struct MessageMember_Foxy
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
unsafe readonly struct MessageMember_Humble
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
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, void*, void> AssignFunction;

    /// <summary>
    /// bool (* resize_function)(void *, size_t size);
    /// </summary>
    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<void*, nint, bool> ResizeFunction;
}

[StructLayout(LayoutKind.Sequential)]
unsafe readonly struct MessageMembers
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

[StructLayout(LayoutKind.Sequential)]
unsafe readonly struct ServiceMembers
{
    public readonly sbyte* ServiceNamespace;
    public readonly sbyte* ServiceName;
    public readonly MessageMembers* RequestMembers;
    public readonly MessageMembers* ResponseMembers;
}
