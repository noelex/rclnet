using Rcl.Introspection;
using Rosidl.Runtime;

namespace Rcl.Internal.Clients;

internal class IntrospectionClient : RclClientBase
{
    private readonly ServiceIntrospection _typesupport;

    public IntrospectionClient(RclNodeImpl node, string serviceName,
        TypeSupportHandle typeSupport, ClientOptions options) : base(node, serviceName, typeSupport, options)
    {
        try
        {
            _typesupport = new ServiceIntrospection(typeSupport);
            RegisterWaitHandle();
        }
        catch { Dispose(); throw; }
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
