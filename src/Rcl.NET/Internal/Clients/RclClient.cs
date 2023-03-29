using Rosidl.Runtime;
using System.Text;

namespace Rcl.Internal.Clients;

internal class RclClient<TService, TRequest, TResponse> : RclClientBase, IRclClient<TRequest, TResponse>
    where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    protected readonly Encoding _textEncoding;

    public unsafe RclClient(
        RclNodeImpl node,
        string serviceName,
        ClientOptions options)
        : base(node, serviceName, TService.GetTypeSupportHandle(), options)
    {
        _textEncoding = options.TextEncoding;
    }

    protected override RosMessageBuffer CreateResponseBuffer()
    {
        return RosMessageBuffer.Create<TResponse>();
    }

    //public Task<TResponse> InvokeAsync(TRequest request, TimeSpan timeout, CancellationToken cancellationToken = default)
    //{
    //    var requestBuffer = RosMessageBuffer.Create<TRequest>();
    //    request.WriteTo(requestBuffer.Data, _textEncoding);

    //    return InvokeAsync(requestBuffer, timeout, cancellationToken).ContinueWith((t, state) =>
    //    {
    //        using (requestBuffer)
    //        using (t.Result)
    //        {
    //            return (TResponse)TResponse.CreateFrom(t.Result.Data, (Encoding)state!);
    //        }
    //    }, _textEncoding, TaskContinuationOptions.ExecuteSynchronously);
    //}

    public async Task<TResponse> InvokeAsync(TRequest request, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        using var requestBuffer = RosMessageBuffer.Create<TRequest>();
        request.WriteTo(requestBuffer.Data, _textEncoding);

        using var responseBuffer = await InvokeAsync(requestBuffer, timeout, cancellationToken).ConfigureAwait(false);
        return (TResponse)TResponse.CreateFrom(responseBuffer.Data, _textEncoding);
    }

    public Task<TResponse> InvokeAsync(TRequest request, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => InvokeAsync(request, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    public Task<TResponse> InvokeAsync(TRequest request, CancellationToken cancellationToken = default)
        => InvokeAsync(request, Timeout.Infinite, cancellationToken);
}