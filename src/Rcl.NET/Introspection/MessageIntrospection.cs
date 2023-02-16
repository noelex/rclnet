using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.Introspection;

internal unsafe class MessageIntrospection
{
    private readonly MessageMembers* _typesupport;

    public MessageIntrospection(TypeSupportHandle typeSupport)
        : this(typeSupport.GetMessageTypeSupport())
    {
    }

    public MessageIntrospection(MessageTypeSupport* ts)
    {
        fixed (byte* id = TypeSupportIdentifier.Introspection)
        {
            ts = ts->Handler(ts, (sbyte*)id);
            _typesupport = (MessageMembers*)ts->Data;
        }
    }

    public unsafe MessageIntrospection(MessageMembers* members)
    {
        _typesupport = members;
    }

    public string Name => StringMarshal.CreatePooledString(_typesupport->MessageName)!;

    public string Namespace => StringMarshal.CreatePooledString(_typesupport->MessageNamespace)!;

    public RosMessageBuffer CreateBuffer()
    {
        var ptr = Marshal.AllocHGlobal(_typesupport->SizeOf);

        // TODO: zero initialization necessary until
        // https://github.com/ros2/ros2/issues/397 is implemented.
        new Span<byte>(ptr.ToPointer(), (int)_typesupport->SizeOf).Clear();

        _typesupport->InitFunction(ptr.ToPointer(), MessageInitialization.All);
        return new RosMessageBuffer(ptr, (buf, state) =>
        {
            var self = (MessageIntrospection)state!;
            self._typesupport->FiniFunction(ptr.ToPointer());
            Marshal.FreeHGlobal(ptr);
        }, this);
    }

    public ref T AsRef<T>(nint msgPtr, int memberIndex)
        where T : unmanaged
    {
        return ref Unsafe.AsRef<T>(GetMemberPointer(msgPtr, memberIndex).ToPointer());
    }

    public nint GetMemberPointer(nint msgPtr, int memberIndex)
    {
        return msgPtr + _typesupport->Members[memberIndex].Offset;
    }

    public string GetMemberName(int memberIndex) => StringMarshal.CreatePooledString(_typesupport->Members[memberIndex].Name)!;

    public FieldType GetMemberType(int memberIndex) => _typesupport->Members[memberIndex].TypeId;

    public int GetMemberOffset(int memberIndex) => _typesupport->Members[memberIndex].Offset;

    public int MemberCount => _typesupport->MemberCount;

    public int SizeOf => (int)_typesupport->SizeOf;
}
