using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime.Interop;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Internal.Services;

/// <summary>
/// Handle requests using native message buffers.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class NativeService<TService, TRequest, TResponse> : IntrospectionService where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    public unsafe NativeService(
        RclNodeImpl node,
        string serviceName,
        INativeServiceHandler handler,
        QosProfile qos)
        : base(node, serviceName, TService.GetTypeSupportHandle(), handler, qos)
    {
    }
}