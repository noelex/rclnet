namespace Rcl.Graph;

public partial class RosGraph
{
    /// <summary>
    /// Checks whether there's an action server with given name available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <returns><see langword="true"/> if the action server is available, otherwise <see langword="false"/>.</returns>
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
    /// Checks whether there's a service server with given name available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <returns><see langword="true"/> if the service server is available, otherwise <see langword="false"/>.</returns>
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

    /// <summary>
    /// Checks whether there's a node with given name available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <returns><see langword="true"/> if the node is available, otherwise <see langword="false"/>.</returns>
    public bool IsNodeAvailable(string nodeName)
    {
        foreach (var name in _nodes.Keys)
        {
            if (name.FullyQualifiedName == nodeName)
            {
                return true;
            }
        }

        return false;
    }

    #region WatchAsync

    /// <summary>
    /// Watch the ROS graph and wait for the graph to enter the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeout">Timeout of the watch.</param>
    /// <param name="state">A custom state object to be passed to the <paramref name="watcher"/> callback.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// <see langword="false"/> if the ROS graph didn't enter the desired state with the timeout. Otherwise, <see langword="true"/>.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, this method will return <see langword="true"/>
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public async Task<bool> TryWatchAsync(Func<RosGraph, RosGraphEvent?, object?, bool> watcher,
        TimeSpan timeout, object? state, CancellationToken cancellationToken = default)
    {
        await Owner.Context.YieldIfNotCurrent();

        if (watcher(this, null, state))
        {
            return true;
        }

        var obs = new GraphWatcher(this, watcher, state);
        using var sub = Subscribe(obs);
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        using var reg = cts.CancelAfter(timeout, _node);

        try
        {
            await obs.Completion.WaitAsync(cts.Token).ConfigureAwait(false);
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

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeoutMilliseconds">Timeout of the watch, in milliseconds.</param>
    /// <param name="state">A custom state object to be passed to the <paramref name="watcher"/> callback.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// <see langword="false"/> if the ROS graph didn't enter the desired state with the timeout. Otherwise, <see langword="true"/>.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, this method will return <see langword="true"/>
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task<bool> TryWatchAsync(Func<RosGraph, RosGraphEvent?, object?, bool> watcher,
        int timeoutMilliseconds, object? state, CancellationToken cancellationToken = default)
        => TryWatchAsync(watcher, TimeSpan.FromMilliseconds(timeoutMilliseconds), state, cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeout">Timeout of the watch.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// <see langword="false"/> if the ROS graph didn't enter the desired state with the timeout. Otherwise, <see langword="true"/>.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, this method will return <see langword="true"/>
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task<bool> TryWatchAsync(Func<RosGraph, RosGraphEvent?, bool> watcher,
        TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWatchAsync(static (g, e, cb) =>
            ((Func<RosGraph, RosGraphEvent?, bool>)cb!)(g, e), timeout, watcher, cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeoutMilliseconds">Timeout of the watch, in milliseconds.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// <see langword="false"/> if the ROS graph didn't enter the desired state with the timeout. Otherwise, <see langword="true"/>.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, this method will return <see langword="true"/>
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task<bool> TryWatchAsync(Func<RosGraph, RosGraphEvent?, bool> watcher,
        int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => TryWatchAsync(watcher, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeout">Timeout of the watch.</param>
    /// <param name="state">A custom state object to be passed to the <paramref name="watcher"/> callback.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// A <see cref="Task"/> represents the asynchronous operation.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, the returned <see cref="Task"/> will complete
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public async Task WatchAsync(Func<RosGraph, RosGraphEvent?, object?, bool> watcher,
        TimeSpan timeout, object? state, CancellationToken cancellationToken = default)
    {
        var success = await TryWatchAsync(watcher, timeout, state, cancellationToken).ConfigureAwait(false);
        if (!success)
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeoutMilliseconds">Timeout of the watch, in milliseconds.</param>
    /// <param name="state">A custom state object to be passed to the <paramref name="watcher"/> callback.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// A <see cref="Task"/> represents the asynchronous operation.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, the returned <see cref="Task"/> will complete
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task WatchAsync(Func<RosGraph, RosGraphEvent?, object?, bool> watcher,
        int timeoutMilliseconds, object? state, CancellationToken cancellationToken = default)
        => WatchAsync(watcher, TimeSpan.FromMilliseconds(timeoutMilliseconds), state, cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait indefinitely until the graph enters the state as specified by the <paramref name="watcher"/>.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="state">A custom state object to be passed to the <paramref name="watcher"/> callback.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// A <see cref="Task"/> represents the asynchronous operation.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, the returned <see cref="Task"/> will complete
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task WatchAsync(Func<RosGraph, RosGraphEvent?, object?, bool> watcher,
        object? state, CancellationToken cancellationToken = default)
        => WatchAsync(watcher, Timeout.InfiniteTimeSpan, state, cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeout">Timeout of the watch.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// A <see cref="Task"/> represents the asynchronous operation.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, the returned <see cref="Task"/> will complete
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task WatchAsync(Func<RosGraph, RosGraphEvent?, bool> watcher,
        TimeSpan timeout, CancellationToken cancellationToken = default)
        => WatchAsync(static (g, e, cb) =>
            ((Func<RosGraph, RosGraphEvent?, bool>)cb!)(g, e), timeout, watcher, cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait until the graph enters the state as specified by the <paramref name="watcher"/>, for a specific period of time.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="timeoutMilliseconds">Timeout of the watch, in milliseconds.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// A <see cref="Task"/> represents the asynchronous operation.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, the returned <see cref="Task"/> will complete
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task WatchAsync(Func<RosGraph, RosGraphEvent?, bool> watcher,
        int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => WatchAsync(watcher, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Watch the ROS graph and wait indefinitely until the graph enters the state as specified by the <paramref name="watcher"/>.
    /// </summary>
    /// <param name="watcher">A predicate to check the state of the ROS graph.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> used for cancelling the asynchronous operation.</param>
    /// <returns>
    /// A <see cref="Task"/> represents the asynchronous operation.
    /// </returns>
    /// <remarks>
    /// The <paramref name="watcher"/> will be called once with a null <see cref="RosGraphEvent"/> before actually watching
    /// the graph. If the <paramref name="watcher"/> returns <see langword="true"/>, the returned <see cref="Task"/> will complete
    /// without waiting. Otherwise, this method will register <see cref="RosGraphEvent"/> and calls the <paramref name="watcher"/>
    /// each time when a event is received, until the <paramref name="watcher"/> returns <see langword="true"/>, or the specified timeout
    /// is reached.
    /// <para>
    /// The <paramref name="watcher"/> is guaranteed to be called on the event loop.
    /// </para>
    /// </remarks>
    public Task WatchAsync(Func<RosGraph, RosGraphEvent?, bool> watcher,
        CancellationToken cancellationToken = default)
        => WatchAsync(watcher, Timeout.InfiniteTimeSpan, cancellationToken);

    #endregion

    #region WaitForServiceServer

    /// <summary>
    /// Try wait for a service server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeout">Timeout of this operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForServiceServerAsync(string serviceName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWatchAsync(static (graph, @event, state) =>
        {
            var name = (string)state!;
            if (@event == null)
            {
                return graph.IsServiceServerAvailable(name);
            }

            return @event is ServerAppearedEvent se && se.Server.Service.Name == name;
        }, timeout, serviceName, cancellationToken);

    /// <summary>
    /// Try wait for a service server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForServiceServerAsync(string serviceName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => TryWaitForServiceServerAsync(serviceName, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Wait until a service server with specific name become available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public async Task WaitForServiceServerAsync(string serviceName, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForServiceServerAsync(serviceName, timeout, cancellationToken).ConfigureAwait(false))
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Wait until a service server with specific name become available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public Task WaitForServiceServerAsync(string serviceName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(serviceName, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Wait until a service server with specific name become available.
    /// </summary>
    /// <param name="serviceName">Name of the service.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    public Task WaitForServiceServerAsync(string serviceName, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(serviceName, Timeout.InfiniteTimeSpan, cancellationToken);

    #endregion WaitForServiceServer

    #region WaitForActionServer

    /// <summary>
    /// Try wait for a action server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForActionServerAsync(string actionName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWatchAsync(static (graph, @event, state) =>
        {
            var name = (string)state!;
            if (@event == null)
            {
                return graph.IsActionServerAvailable(name);
            }

            return @event is ActionServerAppearedEvent se && se.ActionServer.Action.Name == name;
        }, timeout, actionName, cancellationToken);

    /// <summary>
    /// Try wait for a action server with specific name become available, or until timeout.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForActionServerAsync(string actionName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => TryWaitForActionServerAsync(actionName, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Wait until a action server with specific name become available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public async Task WaitForActionServerAsync(string actionName, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForActionServerAsync(actionName, timeout, cancellationToken).ConfigureAwait(false))
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Wait until a action server with specific name become available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    public Task WaitForActionServerAsync(string actionName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => WaitForActionServerAsync(actionName, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Wait until a action server with specific name become available.
    /// </summary>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    public Task WaitForActionServerAsync(string actionName, CancellationToken cancellationToken = default)
        => WaitForActionServerAsync(actionName, Timeout.InfiniteTimeSpan, cancellationToken);

    #endregion WaitForActionServer

    #region WaitForNode

    /// <summary>
    /// Try wait for a node with specific name become available, or until timeout.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeout">Timeout of this operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the node is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForNodeAsync(string nodeName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWatchAsync(static (graph, @event, state) =>
        {
            var name = (string)state!;
            if (@event == null)
            {
                return graph.IsNodeAvailable(name);
            }

            return @event is NodeAppearedEvent se && se.Node.Name.FullyQualifiedName == name;
        }, timeout, nodeName, cancellationToken);

    /// <summary>
    /// Try wait for a node with specific name become available, or until timeout.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the node is available, <see langword="false"/> if timed out.</returns>
    public Task<bool> TryWaitForNodeAsync(string nodeName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => TryWaitForNodeAsync(nodeName, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Wait until a node with specific name become available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The node didn't become available during the wait.</exception>
    public async Task WaitForNodeAsync(string nodeName, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForNodeAsync(nodeName, timeout, cancellationToken).ConfigureAwait(false))
        {
            throw new TimeoutException();
        }
    }

    /// <summary>
    /// Wait until a node with specific name become available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The node didn't become available during the wait.</exception>
    public Task WaitForNodeAsync(string nodeName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => WaitForNodeAsync(nodeName, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    /// <summary>
    /// Wait until a node with specific name become available.
    /// </summary>
    /// <param name="nodeName">Fully qualified name of the node.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    public Task WaitForNodeAsync(string nodeName, CancellationToken cancellationToken = default)
        => WaitForNodeAsync(nodeName, Timeout.InfiniteTimeSpan, cancellationToken);

    #endregion

    private class GraphWatcher : IObserver<RosGraphEvent>
    {
        private readonly RosGraph _graph;
        private readonly TaskCompletionSource _tcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
        private readonly Func<RosGraph, RosGraphEvent?, object?, bool> _predicate;
        private readonly object? _state;

        public GraphWatcher(RosGraph graph, Func<RosGraph, RosGraphEvent?, object?, bool> predicate, object? state)
        {
            _predicate = predicate;
            _state = state;
            _graph = graph;
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
            if (_predicate(_graph, value, _state))
            {
                _tcs.TrySetResult();
            }
        }
    }
}
