using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.Introspection;

internal unsafe class JazzyMessageIntrospection : IMessageIntrospection
{
    private readonly MessageMembers_Jazzy* _typesupport;
    private readonly MessageMember_Jazzy* _members;

    public JazzyMessageIntrospection(TypeSupportHandle typeSupport)
        : this(typeSupport.GetMessageTypeSupport())
    {
    }

    public JazzyMessageIntrospection(MessageTypeSupport* ts)
    {
        fixed (byte* id = TypeSupportIdentifier.Introspection)
        {
            ts = ts->Handler(ts, (sbyte*)id);
            _typesupport = (MessageMembers_Jazzy*)ts->Data;
            _members = (MessageMember_Jazzy*)_typesupport->Members;
        }
    }

    public unsafe JazzyMessageIntrospection(MessageMembers_Jazzy* members)
    {
        _typesupport = members;
        _members = (MessageMember_Jazzy*)_typesupport->Members;
    }

    public string Name => StringMarshal.CreatePooledString(_typesupport->MessageName)!;

    public string Namespace => StringMarshal.CreatePooledString(_typesupport->MessageNamespace)!;

    public RosMessageBuffer CreateBuffer()
    {
        var ptr = Marshal.AllocHGlobal((int)_typesupport->SizeOf);

        // TODO: zero initialization necessary until
        // https://github.com/ros2/ros2/issues/397 is implemented.
        new Span<byte>(ptr.ToPointer(), (int)_typesupport->SizeOf).Clear();

        _typesupport->InitFunction(ptr.ToPointer(), MessageInitialization.All);
        return new RosMessageBuffer(ptr, (buf, state) =>
        {
            var self = (JazzyMessageIntrospection)state!;
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
        return msgPtr + _members[memberIndex].Offset;
    }

    public string GetMemberName(int memberIndex) => StringMarshal.CreatePooledString(_members[memberIndex].Name)!;

    public FieldType GetMemberType(int memberIndex) => _members[memberIndex].TypeId;

    public int GetMemberOffset(int memberIndex) => _members[memberIndex].Offset;

    public int MemberCount => (int)_typesupport->MemberCount;

    public int SizeOf => (int)_typesupport->SizeOf;
}