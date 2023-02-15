using Rcl.Introspection;
using Rcl.Qos;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Internal.Clients;

internal class IntrospectionRclClient : RclClientBase
{
    private readonly ServiceIntrospection _typesupport;

    public IntrospectionRclClient(RclNodeImpl node, string serviceName, TypeSupportHandle typeSupport, QosProfile qos) : base(node, serviceName, typeSupport, qos)
    {
        _typesupport = new ServiceIntrospection(typeSupport);
    }

    protected override RosMessageBuffer CreateResponseBuffer()
    {
        return _typesupport.Response.CreateBuffer();
    }

    public RosMessageBuffer CreateRequestBuffer()
    {
        return _typesupport.Request.CreateBuffer();
    }

   
}
