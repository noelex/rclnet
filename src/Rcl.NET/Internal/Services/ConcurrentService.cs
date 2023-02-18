﻿using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Reflection.PortableExecutable;
using System.Text;

namespace Rcl.Internal.Services;

/// <summary>
/// This class allows handling multiple concurrent service requests asynchronously.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class ConcurrentService<TService, TRequest, TResponse> 
    : ConcurrentIntrospectionService
    where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    public unsafe ConcurrentService(
        RclNodeImpl node,
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
        QosProfile qos,
        Encoding textEncoding)
        : base(node, serviceName, new Handler(handler, textEncoding), TService.GetTypeSupportHandle(), qos)
    {
    }

    private class Handler : IConcurrentNativeServiceHandler
    {
        private readonly Encoding _textEncoding;
        private readonly IConcurrentServiceHandler<TRequest, TResponse> _handler;

        public Handler(IConcurrentServiceHandler<TRequest, TResponse> handler, Encoding textEncoding)
        {
            _handler= handler;
            _textEncoding = textEncoding;
        }

        public async Task ProcessRequestAsync(
            RosMessageBuffer request, RosMessageBuffer response, CancellationToken cancellationToken = default)
        {
            var req = (TRequest)TRequest.CreateFrom(request.Data, _textEncoding);
            var resp = await _handler.ProcessRequestAsync(req, cancellationToken);
            resp.WriteTo(response.Data, _textEncoding);
        }
    }

}
