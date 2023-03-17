using Rosidl.Runtime;

namespace Rcl;

/// <summary>
/// Represents an ROS service for handling service requests.
/// </summary>
public interface IRclService : IRclObject
{
    /// <summary>
    /// Determine whether current <see cref="IRclService"/> instance is valid.
    /// </summary>
    bool IsValid { get; }

    /// <summary>
    /// Gets the name of the service.
    /// </summary>
    string Name { get; }
}

/// <summary>
/// Represents a handler which is handles service requests on the <see cref="RclContext"/> event loop.
/// </summary>
/// <remarks>
/// The handler will be invoked on the <see cref="RclContext"/> event loop to process incoming requests.
/// Performing blocking operations in the handler will affect the responsiveness of the event loop.
/// If the handler issues blocking calls or performs CPU intensive calculations, it's recommended to
/// use <see cref="IConcurrentServiceHandler{TRequest, TResponse}"/> instead.
/// </remarks>
/// <typeparam name="TRequest">Type of the request messages.</typeparam>
/// <typeparam name="TResponse">Type of the response messages.</typeparam>
public interface IServiceHandler<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    /// <summary>
    /// Process the incoming request.
    /// </summary>
    /// <param name="request">The request message received from the client.</param>
    /// <returns>The response message to be sent to the client.</returns>
    TResponse ProcessRequest(TRequest request);
}

/// <summary>
/// Same as <see cref="IServiceHandler{TRequest,TResponse}"/>, but allow
/// implementations to operate on native ROS message buffers
/// directy, without creating request and response object.
/// </summary>
/// <remarks>
/// This is especially useful when dealing with complex messages or messages
/// has large array fields. Operaing directly on native buffers would effectively
/// lower the cost of copying and reduce managed heap allocation.
/// <para>
/// The handler will be invoked on the <see cref="RclContext"/> event loop to process incoming requests.
/// Performing blocking operations in the handler will affect the responsiveness of the event loop.
/// If the handler issues blocking calls or performs CPU intensive calculations, it's recommended to
/// use <see cref="IConcurrentNativeServiceHandler"/> instead.
/// </para>
/// </remarks>
public interface INativeServiceHandler
{
    /// <summary>
    /// Process the incoming request.
    /// </summary>
    /// <remarks>
    /// <para>
    /// The <paramref name="request"/> and <paramref name="response"/> buffers are only valid during the call to <see cref="ProcessRequest"/>. 
    /// Do not store reference or try to access these buffers from outside of the method scope.
    /// </para>
    /// <para>
    /// Additionally, the ownership of the buffers is <i><b>NOT</b></i> transferred to this method.
    /// Disposing these buffers may cause unexpected behavior.
    /// </para>
    /// </remarks>
    /// <param name="request"></param>
    /// <param name="response"></param>
    void ProcessRequest(RosMessageBuffer request, RosMessageBuffer response);
}

/// <summary>
/// Represents a handler which is able to handle multiple service requests concurrently.
/// </summary>
/// <remarks>
/// The handler will start its execution on the <see cref="RclContext"/> event loop to process incoming requests.
/// If the handler issues blocking calls or performs CPU intensive calculations, <see cref="Task.Yield"/> should
/// be called before performing any blocking operation, or you can use <see cref="Task.Run(Action)"/> to perform the
/// blocking task in a background thread.
/// </remarks>
/// <typeparam name="TRequest">Type of the request messages.</typeparam>
/// <typeparam name="TResponse">Type of the response messages.</typeparam>
public interface IConcurrentServiceHandler<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    /// <summary>
    /// Process the incoming request asynchronously.
    /// </summary>
    /// <param name="request">A <typeparamref name="TRequest"/> object containing the request data.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> which will be canceled when the corresponding <see cref="IRclService"/> is being disposed.</param>
    /// <returns>A <typeparamref name="TResponse"/> object containing the response data.</returns>
    Task<TResponse> ProcessRequestAsync(TRequest request, CancellationToken cancellationToken = default);
}

/// <summary>
/// Same as <see cref="INativeServiceHandler"/>, but handles requests concurrently.
/// </summary>
/// <remarks>
/// The handler will start its execution on the <see cref="RclContext"/> event loop to process incoming requests.
/// If the handler issues blocking calls or performs CPU intensive calculations, <see cref="Task.Yield"/> should
/// be called before performing any blocking operation, or you can use <see cref="Task.Run(Action)"/> to perform the
/// blocking task in a background thread.
/// </remarks>
public interface IConcurrentNativeServiceHandler
{
    /// <summary>
    /// Process the incoming request asynchronously.
    /// </summary>
    /// <param name="request">A <see cref="RosMessageBuffer"/> object containing the request data.</param>
    /// <param name="response">A <see cref="RosMessageBuffer"/> to be filled with response data.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> which will be canceled when the corresponding <see cref="IRclService"/> is being disposed.</param>
    /// <remarks>
    /// <para>
    /// The <paramref name="request"/> and <paramref name="response"/> buffers are only valid during the call to <see cref="ProcessRequestAsync"/>. 
    /// Do not store reference or try to access these buffers from outside of the method scope.
    /// </para>
    /// <para>
    /// Additionally, the ownership of the buffers is <i><b>NOT</b></i> transferred to this method.
    /// Disposing these buffers may cause unexpected behavior.
    /// </para>
    /// </remarks>
    Task ProcessRequestAsync(RosMessageBuffer request, RosMessageBuffer response, CancellationToken cancellationToken = default);
}