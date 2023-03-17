using Rosidl.Runtime;

namespace Rcl;

/// <summary>
/// An ROS service client for performing remote service calls.
/// </summary>
/// <typeparam name="TRequest">Type of the request message.</typeparam>
/// <typeparam name="TResponse">Type of the response message.</typeparam>
public interface IRclClient<TRequest, TResponse> : IRclObject
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    /// <summary>
    /// Determine whether current <see cref="IRclClient{TRequest, TResponse}"/> is valid.
    /// </summary>
    bool IsValid { get; }

    /// <summary>
    /// Determine whether there's any server available to process client requests.
    /// </summary>
    bool IsServerAvailable { get; }

    /// <summary>
    /// Gets the name of the service being invoked.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Try wait for the server become available, or until timeout.
    /// </summary>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    Task<bool> TryWaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Try wait for the server become available, or until timeout.
    /// </summary>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    Task<bool> TryWaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Wait until the server become available.
    /// </summary>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    Task WaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Wait until the server become available.
    /// </summary>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    Task WaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Wait until the server become available.
    /// </summary>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    Task WaitForServerAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Initiate a service request by sending a pre-allocated <see cref="RosMessageBuffer"/> and wait for the response asynchronously.
    /// </summary>
    /// <param name="request">A <see cref="RosMessageBuffer"/> containing the request message to be sent.</param>
    /// <param name="cancellationToken">A <paramref name="cancellationToken"/> to cancel the request.</param>
    /// <returns>A <see cref="RosMessageBuffer"/> containing the response message.</returns>
    /// <remarks>
    /// This method does not take the ownership of the <paramref name="request"/> and response <see cref="RosMessageBuffer"/>.
    /// It's the caller's responsibility to make sure these buffers are disposed appropiately.
    /// <para>
    /// In contrast to overloads taking <typeparamref name="TRequest"/> and returning <typeparamref name="TResponse"/>,
    /// this method performs neither managed heap allocation nor buffer copying to send or receive messages, thus can achieve
    /// higher throughput, especially when the service call is frequently performed or the message contains complex fields.
    /// </para>
    /// </remarks>
    Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, CancellationToken cancellationToken = default);

    /// <summary>
    /// Initiate a service request by sending a pre-allocated <see cref="RosMessageBuffer"/> and wait for the response asynchronously.
    /// </summary>
    /// <param name="request">A <see cref="RosMessageBuffer"/> containing the request message to be sent.</param>
    /// <param name="timeoutMilliseconds">Timeout of the request in milliseconds.</param>
    /// <param name="cancellationToken">A <paramref name="cancellationToken"/> to cancel the request.</param>
    /// <returns>A <see cref="RosMessageBuffer"/> containing the response message.</returns>
    /// <remarks>
    /// This method does not take the ownership of the <paramref name="request"/> and response <see cref="RosMessageBuffer"/>.
    /// It's the caller's responsibility to make sure these buffers are disposed appropiately.
    /// <para>
    /// In contrast to overloads taking <typeparamref name="TRequest"/> and returning <typeparamref name="TResponse"/>,
    /// this method performs neither managed heap allocation nor buffer copying to send or receive messages, thus can achieve
    /// higher throughput, especially when the service call is frequently performed or the message contains complex fields.
    /// </para>
    /// </remarks>
    Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Initiate a service request by sending a pre-allocated <see cref="RosMessageBuffer"/> and wait for the response asynchronously.
    /// </summary>
    /// <param name="request">A <see cref="RosMessageBuffer"/> containing the request message to be sent.</param>
    /// <param name="timeout">Timeout of the request.</param>
    /// <param name="cancellationToken">A <paramref name="cancellationToken"/> to cancel the request.</param>
    /// <returns>A <see cref="RosMessageBuffer"/> containing the response message.</returns>
    /// <remarks>
    /// This method does not take the ownership of the <paramref name="request"/> and response <see cref="RosMessageBuffer"/>.
    /// It's the caller's responsibility to make sure these buffers are disposed appropiately.
    /// <para>
    /// In contrast to overloads taking <typeparamref name="TRequest"/> and returning <typeparamref name="TResponse"/>,
    /// this method performs neither managed heap allocation nor buffer copying to send or receive messages, thus can achieve
    /// higher throughput, especially when the service call is frequently performed or the message contains complex fields.
    /// </para>
    /// </remarks>
    Task<RosMessageBuffer> InvokeAsync(RosMessageBuffer request, TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Initiate a service request by sending a <typeparamref name="TRequest"/> message and wait for the response asynchronously.
    /// </summary>
    /// <param name="request">A <typeparamref name="TRequest"/> object containing the request message to be sent.</param>
    /// <param name="cancellationToken">A <paramref name="cancellationToken"/> to cancel the request.</param>
    /// <returns>A <typeparamref name="TResponse"/> object containing the response message.</returns>
    Task<TResponse> InvokeAsync(TRequest request, CancellationToken cancellationToken = default);

    /// <summary>
    /// Initiate a service request by sending a <typeparamref name="TRequest"/> message and wait for the response asynchronously.
    /// </summary>
    /// <param name="request">A <typeparamref name="TRequest"/> object containing the request message to be sent.</param>
    /// <param name="timeoutMilliseconds">Timeout of the request.</param>
    /// <param name="cancellationToken">A <paramref name="cancellationToken"/> to cancel the request.</param>
    /// <returns>A <typeparamref name="TResponse"/> object containing the response message.</returns>
    Task<TResponse> InvokeAsync(TRequest request, int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Initiate a service request by sending a <typeparamref name="TRequest"/> message and wait for the response asynchronously.
    /// </summary>
    /// <param name="request">A <typeparamref name="TRequest"/> object containing the request message to be sent.</param>
    /// <param name="timeout">Timeout of the request.</param>
    /// <param name="cancellationToken">A <paramref name="cancellationToken"/> to cancel the request.</param>
    /// <returns>A <typeparamref name="TResponse"/> object containing the response message.</returns>
    Task<TResponse> InvokeAsync(TRequest request, TimeSpan timeout, CancellationToken cancellationToken = default);
}