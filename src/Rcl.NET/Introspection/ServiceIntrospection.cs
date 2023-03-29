using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Introspection;

internal unsafe class ServiceIntrospection
{
    private readonly ServiceMembers* _typesupport;

    public ServiceIntrospection(TypeSupportHandle typeSupport)
        : this(typeSupport.GetServiceTypeSupport())
    {
    }

    public ServiceIntrospection(ServiceTypeSupport* ts)
    {
        TypeSupportHandle = ts;
        fixed (byte* id = TypeSupportIdentifier.Introspection)
        {
            ts = ts->Handler(ts, (sbyte*)id);
            var serviceMembers = (ServiceMembers*)ts->Data;
            Request = MessageIntrospection.Create(serviceMembers->RequestMembers);
            Response = MessageIntrospection.Create(serviceMembers->ResponseMembers);
            _typesupport = serviceMembers;
        }
    }

    public ServiceTypeSupport* TypeSupportHandle { get; }

    public string Name => StringMarshal.CreatePooledString(_typesupport->ServiceName)!;

    public string Namespace => StringMarshal.CreatePooledString(_typesupport->ServiceNamespace)!;

    public IMessageIntrospection Request { get; }

    public IMessageIntrospection Response { get; }
}
