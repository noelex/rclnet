namespace Rcl;

/// <summary>
/// Represents an RCL waitable object.
/// </summary>
public interface IRclWaitObject : IRclObject
{
    /// <summary>
    /// Asynchronously wait for current <see cref="IRclWaitObject"/> instance to be triggered once.
    /// </summary>
    /// <remarks>
    /// Continuations may or may not run on the <see cref="RclContext"/> event loop which the <see cref="IRclWaitObject"/> belongs to, depdending on
    /// the captured <see cref="SynchronizationContext"/>.
    /// <para>
    /// If there is no captured <see cref="SynchronizationContext"/>, 
    /// contiuations will run synchronously on the event loop if <paramref name="runContinuationAsynchronously"/> is set to <see langword="false"/>,
    /// and will run asynchronously on a <see cref="ThreadPool"/> thread if <paramref name="runContinuationAsynchronously"/> is set to <see langword="true"/>.
    /// </para>
    /// <para>
    /// If there is a captured <see cref="SynchronizationContext"/>, then the contiuations will run asynchronously on the captured <see cref="SynchronizationContext"/>
    /// regardless of the value of the  <paramref name="runContinuationAsynchronously"/> parameter.
    /// </para>
    /// </remarks>
    /// <param name="runContinuationAsynchronously">
    /// Whether to force continuations to run asynchronously.
    /// <para>
    /// Setting to <see langword="true"/> will prevent contiuations from running synchronously on the <see cref="RclContext"/> event loop.
    /// </para>
    /// </param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> to cancel the asynchronous wait.</param>
    /// <returns>
    /// A <see cref="ValueTask"/> representing the completion of the asynchronous wait.
    /// </returns>
    ValueTask WaitOneAsync(bool runContinuationAsynchronously, CancellationToken cancellationToken = default);

    /// <summary>
    /// Asynchronously wait for current <see cref="IRclWaitObject"/> instance to be triggered once, without forcing continuations to run asynchronously.
    /// </summary>
    /// <remarks>
    /// Calling this method has the same effect as calling <see cref="WaitOneAsync(bool, CancellationToken)"/> with <c>runContinuationAsynchronously</c> = <see langword="true"/>.
    /// </remarks>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> to cancel the asynchronous wait.</param>
    /// <returns>
    /// A <see cref="ValueTask"/> representing the completion of the asynchronous wait.
    /// </returns>
    ValueTask WaitOneAsync(CancellationToken cancellationToken = default);
}