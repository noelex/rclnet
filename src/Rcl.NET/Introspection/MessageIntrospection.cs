using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Introspection;

interface IMessageIntrospection
{
    string Name { get; }

    string Namespace { get; }

    RosMessageBuffer CreateBuffer();

    ref T AsRef<T>(nint msgPtr, int memberIndex)
        where T : unmanaged;

    nint GetMemberPointer(nint msgPtr, int memberIndex);

    string GetMemberName(int memberIndex);

    FieldType GetMemberType(int memberIndex);

    int GetMemberOffset(int memberIndex);

    int MemberCount { get; }

    int SizeOf { get; }
}

static class MessageIntrospection
{
    public unsafe static IMessageIntrospection Create(MessageTypeSupport* typesupport)
    {
        if (RosEnvironment.IsFoxy)
        {
            return new FoxyMessageIntrospection(typesupport);
        }
        else if (RosEnvironment.IsHumble)
        {
            return new HumbleMessageIntrospection(typesupport);
        }
        else
        {
            throw new NotSupportedException();
        }
    }

    public unsafe static IMessageIntrospection Create(TypeSupportHandle typesupport)
        => Create(typesupport.GetMessageTypeSupport());

    public unsafe static IMessageIntrospection Create(MessageMembers* messageMembers)
    {
        if (RosEnvironment.IsFoxy)
        {
            return new FoxyMessageIntrospection(messageMembers);
        }
        else if (RosEnvironment.IsHumble)
        {
            return new HumbleMessageIntrospection(messageMembers);
        }
        else
        {
            throw new NotSupportedException();
        }
    }
}