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
    /// Sets the default timeout of <see cref="InvokeAsync(RosMessageBuffer, CancellationToken)"/> and <see cref="InvokeAsync(TRequest, CancellationToken)"/>.
    /// <para>
    /// Defaults to 60 seconds.
    /// </para>
    /// </summary>
    TimeSpan DefaultRequestTimeout { get; set; }

    /// <summary>
    /// Determine whether current <see cref="IRclClient{TRequest, TResponse}"/> is valid.
    /// </summary>
    bool IsValid { get; }

    bool IsServerAvailable { get; }

    /// <summary>
    /// Gets the name of the service being invoked.
    /// </summary>
    string? Name { get; }

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
    /// <param name="timeoutMilliseconds">Timeout of the request.</param>
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