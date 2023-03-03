using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Graph;

public partial class RosGraph
{
    /// <summary>
    /// Checks whether there's an action server with given name available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <returns><see langword="true"/> if action server is available, otherwise <see langword="false"/>.</returns>
    public bool IsActionServerAvailable(string actionName)
    {
        foreach (var action in Actions)
        {
            if (action.Name == actionName)
            {
                return action.Servers.Count > 0;
            }
        }

        return false;
    }

    /// <summary>
    /// Checks whether there's an service server with given name available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <returns><see langword="true"/> if service server is available, otherwise <see langword="false"/>.</returns>
    public bool IsServiceServerAvailable(string serviceName)
    {
        foreach (var service in Services)
        {
            if (service.Name == serviceName)
            {
                return service.Servers.Count > 0;
            }
        }

        return false;
    }

    internal async Task<bool> TryWaitForEventAsync(Func<RosGraphEvent, object?, bool> predicate,
        object? state, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        var obs = new Observer(predicate, state);
        using var sub = Subscribe(obs);
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        using var reg = cts.CancelAfter(timeoutMilliseconds, _node);

        try
        {
            await obs.Completion.WaitAsync(cts.Token);
            return true;
        }
        catch (OperationCanceledException)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                throw;
            }

            return false;
        }
    }

    #region WaitForServiceServer

    /// <summary>
    /// Try wait for a service server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public async Task<bool> TryWaitForServiceServerAsync(string serviceName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        // Yield back to event loop to make sure we have
        // consistent view on the graph during query.
        if (!_node.Context.IsCurrent) await _node.Context.Yield();

        if (IsServiceServerAvailable(serviceName))
        {
            return true;
        }

        return await TryWaitForEventAsync(
             static (e, s) => e is ServerAppearedEvent se && se.Server.Service.Name == (string)s!,
             serviceName, timeoutMilliseconds, cancellationToken);
    }

    /// <summary>
    /// Try wait for a service server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeout">Timeout of this operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForServiceServerAsync(string serviceName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWaitForServiceServerAsync(serviceName, (int)timeout.TotalMilliseconds, cancellationToken);

    /// <summary>
    /// Wait until a service server with specific name become available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public async Task WaitForServiceServerAsync(string serviceName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForServiceServerAsync(serviceName, timeoutMilliseconds, cancellationToken))
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Wait until a service server with specific name become available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public Task WaitForServiceServerAsync(string serviceName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(serviceName, (int)timeout.TotalMilliseconds, cancellationToken);

    /// <summary>
    /// Wait until a service server with specific name become available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    public Task WaitForServiceServerAsync(string serviceName, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(serviceName, -1, cancellationToken);

    #endregion WaitForServiceServer

    #region WaitForActionServer

    /// <summary>
    /// Try wait for a action server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public async Task<bool> TryWaitForActionServerAsync(string actionName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        // Yield back to event loop to make sure we have
        // consistent view on the graph during query.
        if (!_node.Context.IsCurrent) await _node.Context.Yield();

        if (IsActionServerAvailable(actionName))
        {
            return true;
        }

        return await TryWaitForEventAsync(
             static (e, s) => e is ActionServerAppearedEvent se && se.ActionServer.Action.Name == (string)s!,
             actionName, timeoutMilliseconds, cancellationToken);
    }

    /// <summary>
    /// Try wait for a action server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForActionServerAsync(string actionName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWaitForActionServerAsync(actionName, (int)timeout.TotalMilliseconds, cancellationToken);

    /// <summary>
    /// Wait until a action server with specific name become available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public async Task WaitForActionServerAsync(string actionName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForActionServerAsync(actionName, timeoutMilliseconds, cancellationToken))
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Wait until a action server with specific name become available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public Task WaitForActionServerAsync(string actionName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(actionName, (int)timeout.TotalMilliseconds, cancellationToken);

    /// <summary>
    /// Wait until a action server with specific name become available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    public Task WaitForActionServerAsync(string actionName, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(actionName, -1, cancellationToken);

    #endregion WaitForActionServer

    #region WaitForNode

    /// <summary>
    /// Try wait for a node with specific name become available, or until timeout.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the node is available, <see langword="false"/> if timed out.</returns>
    public async Task<bool> TryWaitForNodeAsync(string nodeName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        // Yield back to event loop to make sure we have
        // consistent view on the graph during query.
        if (!_node.Context.IsCurrent) await _node.Context.Yield();

        foreach (var node in _nodes)
        {
            if (node.Value.Name.FullyQualifiedName == nodeName)
            {
                return true;
            }
        }

        return await TryWaitForEventAsync(
             static (e, s) => e is NodeAppearedEvent nae && nae.Node.Name.FullyQualifiedName == (string)s!,
             nodeName, timeoutMilliseconds, cancellationToken);
    }

    /// <summary>
    /// Try wait for a node with specific name become available, or until timeout.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeout">Timeout of this operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the node is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForNodeAsync(string nodeName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWaitForNodeAsync(nodeName, (int)timeout.TotalMilliseconds, cancellationToken);

    /// <summary>
    /// Wait until a node with specific name become available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The node didn't become available during the wait.</exception>
    public async Task WaitForNodeAsync(string nodeName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForNodeAsync(nodeName, timeoutMilliseconds, cancellationToken))
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Wait until a node with specific name become available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The node didn't become available during the wait.</exception>
    public Task WaitForNodeAsync(string nodeName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => WaitForNodeAsync(nodeName, (int)timeout.TotalMilliseconds, cancellationToken);

    /// <summary>
    /// Wait until a node with specific name become available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    public Task WaitForNodeAsync(string nodeName, CancellationToken cancellationToken = default)
        => WaitForNodeAsync(nodeName, -1, cancellationToken);

    #endregion

    private class Observer : IObserver<RosGraphEvent>
    {
        private readonly TaskCompletionSource _tcs = new();
        private readonly Func<RosGraphEvent, object?, bool> _predicate;
        private readonly object? _state;

        public Observer(Func<RosGraphEvent, object?, bool> predicate, object? state)
        {
            _predicate = predicate;
            _state = state;
        }

        public Task Completion => _tcs.Task;

        public void OnCompleted()
        {
            _tcs.TrySetException(new ObjectDisposedException(typeof(RosGraph).Name));
        }

        public void OnError(Exception error)
        {

        }

        public void OnNext(RosGraphEvent value)
        {
            if (_predicate(value, _state))
            {
                _tcs.TrySetResult();
            }
        }
    }
}
