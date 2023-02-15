using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Introspection;

internal unsafe class ServiceIntrospection
{
    private readonly ServiceMembers* _typesupport;

    public  ServiceIntrospection(TypeSupportHandle typeSupport)
        :this(typeSupport.GetServiceTypeSupport())
    {
    }

    public ServiceIntrospection(ServiceTypeSupport* ts)
    {
        fixed (byte* id = TypeSupportIdentifier.Introspection)
        {
            ts = ts->Handler(ts, (sbyte*)id);
            var serviceMembers = (ServiceMembers*)ts->Data;
            Request = new(serviceMembers->RequestMembers);
            Response = new(serviceMembers->ResponseMembers);
            _typesupport = serviceMembers;
        }
    }

    public string Name => StringMarshal.CreatePooledString(_typesupport->ServiceName)!;

    public string Namespace => StringMarshal.CreatePooledString(_typesupport->ServiceNamespace)!;

    public MessageIntrospection Request { get; }

    public MessageIntrospection Response { get; }
}
