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

    private async Task<bool> TryWaitForConditionAsync(Func<RosGraphEvent, object?, bool> predicate,
        object? state, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        var obs = new Observer(predicate, state);
        using var sub = Subscribe(obs);
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        cts.CancelAfter(timeoutMilliseconds);

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

    public Task<bool> TryWaitForServiceServerAsync(string serviceName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (IsServiceServerAvailable(serviceName))
        {
            return Task.FromResult(true);
        }

        return TryWaitForConditionAsync(
             static (e, s) => e is ServerEstablishedEvent se && se.Server.Service.Name == (string)s!,
             serviceName, timeoutMilliseconds, cancellationToken);
    }

    public Task<bool> TryWaitForServiceServerAsync(string serviceName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWaitForServiceServerAsync(serviceName, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<bool> TryWaitForServiceServerAsync(string serviceName, CancellationToken cancellationToken = default)
        => TryWaitForServiceServerAsync(serviceName, -1, cancellationToken);

    public async Task<bool> WaitForServiceServerAsync(string serviceName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (!await TryWaitForServiceServerAsync(serviceName, timeoutMilliseconds, cancellationToken))
        {
            throw new TimeoutException();
        }

        return true;
    }

    public Task WaitForServiceServerAsync(string serviceName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(serviceName, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task WaitForServiceServerAsync(string serviceName, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(serviceName, -1, cancellationToken);

    public Task<bool> TryWaitForActionServerAsync(string actionName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (IsActionServerAvailable(actionName))
        {
            return Task.FromResult(true);
        }

        return TryWaitForConditionAsync(
             static (e, s) => e is ActionServerEstablishedEvent se && se.ActionServer.Action.Name == (string)s!,
             actionName, timeoutMilliseconds, cancellationToken);
    }

    public Task<bool> TryWaitForActionServerAsync(string actionName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => TryWaitForActionServerAsync(actionName, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<bool> TryWaitForActionServerAsync(string actionName, CancellationToken cancellationToken = default)
        => TryWaitForActionServerAsync(actionName, -1, cancellationToken);

    public async Task<bool> WaitForActionServerAsync(string actionName, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        if (!await WaitForActionServerAsync(actionName, timeoutMilliseconds, cancellationToken))
        {
            throw new TimeoutException();
        }

        return true;
    }

    public Task WaitForActionServerAsync(string actionName, TimeSpan timeout, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(actionName, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task WaitForActionServerAsync(string actionName, CancellationToken cancellationToken = default)
        => WaitForServiceServerAsync(actionName, -1, cancellationToken);

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
