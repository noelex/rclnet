using System.Runtime.CompilerServices;

namespace Rcl;

/// <summary>Provides an awaitable context for switching into a target environment.</summary>
/// <remarks>This type is intended for compiler use only.</remarks>
public readonly struct YieldAwaiter : ICriticalNotifyCompletion
{
    private readonly SynchronizationContext? _preferredContext;
    private readonly bool _isCompleted;

    internal YieldAwaiter(SynchronizationContext? preferredContext, bool isCompleted)
    {
        _preferredContext = preferredContext;
        _isCompleted = isCompleted;
    }

    /// <summary>Gets whether a yield is not required.</summary>
    /// <remarks>This property is intended for compiler user rather than use directly in code.</remarks>
    public bool IsCompleted => _isCompleted;  // yielding is always required for YieldAwaiter, hence false

    /// <summary>Posts the <paramref name="continuation"/> back to the current context.</summary>
    /// <param name="continuation">The action to invoke asynchronously.</param>
    /// <exception cref="System.ArgumentNullException">The <paramref name="continuation"/> argument is null (Nothing in Visual Basic).</exception>
    public void OnCompleted(Action continuation)
    {
        QueueContinuation(continuation, flowContext: true);
    }

    /// <summary>Posts the <paramref name="continuation"/> back to the current context.</summary>
    /// <param name="continuation">The action to invoke asynchronously.</param>
    /// <exception cref="System.ArgumentNullException">The <paramref name="continuation"/> argument is null (Nothing in Visual Basic).</exception>
    public void UnsafeOnCompleted(Action continuation)
    {
        QueueContinuation(continuation, flowContext: false);
    }

    private void QueueContinuation(Action continuation, bool flowContext)
    {
        ArgumentNullException.ThrowIfNull(continuation);

        if (_preferredContext != null)
        {
            _preferredContext.Post(s_sendOrPostCallbackRunAction, continuation);
        }
        else
        {
            if (flowContext)
            {
                ThreadPool.QueueUserWorkItem(s_waitCallbackRunAction, continuation);
            }
            else
            {
                ThreadPool.UnsafeQueueUserWorkItem(s_waitCallbackRunAction, continuation);
            }
        }
    }

    private static readonly WaitCallback s_waitCallbackRunAction = RunAction;

    private static readonly SendOrPostCallback s_sendOrPostCallbackRunAction = RunAction;

    /// <summary>Runs an Action delegate provided as state.</summary>
    /// <param name="state">The Action delegate to invoke.</param>
    private static void RunAction(object? state) { ((Action)state!)(); }

    /// <summary>Ends the await operation.</summary>
    public void GetResult() { } // Nop. It exists purely because the compiler pattern demands it.

    /// <summary>Gets an awaiter for this <see cref="YieldAwaitable"/>.</summary>
    /// <returns>An awaiter for this awaitable.</returns>
    /// <remarks>This method is intended for compiler use rather than use directly in code.</remarks>
    public YieldAwaiter GetAwaiter() { return this; }
}
