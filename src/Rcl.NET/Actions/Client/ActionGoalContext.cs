using Rosidl.Runtime;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Channels;

namespace Rcl.Actions.Client;

internal class ActionGoalContext<TResult, TFeedback> : ActionGoalContextBase, IActionGoalContext<TResult, TFeedback>
    where TFeedback : IActionFeedback
    where TResult : IActionResult
{
    private readonly Channel<TFeedback> _feedbackChannel;
    private readonly object _observersGate = new();
    private readonly Dictionary<int, IObserver<TFeedback>> _observers = new();
    private IObserver<TFeedback>[] _observerSnapshot = Array.Empty<IObserver<TFeedback>>();
    private readonly Encoding _textEncoding;

    private bool _completed;
    private int _activeDispatches;
    private List<IObserver<TFeedback>>? _completionObservers;
    private int _channelReaders = 0, _subscriberId;

    public ActionGoalContext(Guid goalId, IActionClientImpl actionClient, Encoding textEncoding)
        : base(goalId, actionClient)
    {
        _textEncoding = textEncoding;

        var opts = new BoundedChannelOptions(actionClient.Options.QueueSize)
        {
            AllowSynchronousContinuations = actionClient.Options.AllowSynchronousContinuations,
            SingleReader = false,
            SingleWriter = true,
            FullMode = actionClient.Options.FullMode
        };
        _feedbackChannel = Channel
            .CreateBounded<TFeedback>(opts);
    }

    public override bool HasFeedbackListeners => Volatile.Read(ref _channelReaders) > 0 || Volatile.Read(ref _observerSnapshot).Length > 0;

    public override void OnFeedbackReceived(RosMessageBuffer feedback)
    {
        TFeedback msg;
        using (feedback)
        {
            msg = (TFeedback)TFeedback.CreateFrom(feedback.Data, _textEncoding);
        }

        IObserver<TFeedback>[] observers;

        lock (_observersGate)
        {
            if (_completed)
            {
                return;
            }

            _activeDispatches++;
            observers = _observerSnapshot;
        }

        try
        {
            _feedbackChannel.Writer.TryWrite(msg);
            // An admitted snapshot finishes before completion, even if a callback closes the goal.
            foreach (var observer in observers)
            {
                observer.OnNext(msg);
            }
        }
        finally
        {
            List<IObserver<TFeedback>>? completed = null;

            lock (_observersGate)
            {
                if (--_activeDispatches == 0)
                {
                    completed = _completionObservers;
                    _completionObservers = null;
                }
            }

            CompleteFeedback(completed);
        }
    }

    public async IAsyncEnumerable<TFeedback> ReadFeedbacksAsync([EnumeratorCancellation] CancellationToken cancellationToken)
    {
        Interlocked.Increment(ref _channelReaders);

        try
        {
            await foreach (var buffer in _feedbackChannel.Reader.ReadAllAsync(cancellationToken).ConfigureAwait(false))
            {
                yield return buffer;
            }
        }
        finally
        {
            Interlocked.Decrement(ref _channelReaders);
        }
    }

    public IDisposable Subscribe(IObserver<TFeedback> observer)
    {
        lock (_observersGate)
        {
            if (!_completed)
            {
                var id = ++_subscriberId;
                _observers[id] = observer;
                Volatile.Write(ref _observerSnapshot, _observers.Values.ToArray());
                return new Subscription(id, this);
            }

            if (_activeDispatches != 0)
            {
                // This observer may also belong to an in-flight snapshot from an earlier subscription.
                _completionObservers!.Add(observer);
                return Subscription.Empty;
            }
        }

        observer.OnCompleted();
        return Subscription.Empty;
    }

    private void Unsubscribe(int id)
    {
        lock (_observersGate)
        {
            if (_observers.Remove(id))
            {
                Volatile.Write(ref _observerSnapshot, _observers.Values.ToArray());
            }
        }
    }

    protected override void OnGoalStateChanged(ActionGoalStatus state)
    {
        if (state is ActionGoalStatus.Canceled or ActionGoalStatus.Succeeded or ActionGoalStatus.Aborted)
        {
            OnDispose();
        }
    }

    protected override void OnDispose()
    {
        List<IObserver<TFeedback>>? observers;

        lock (_observersGate)
        {
            if (_completed)
            {
                return;
            }

            _completed = true;
            observers = new List<IObserver<TFeedback>>(_observerSnapshot);

            if (_activeDispatches != 0)
            {
                _completionObservers = observers;
                observers = null;
            }

            _observers.Clear();
            Volatile.Write(ref _observerSnapshot, Array.Empty<IObserver<TFeedback>>());
        }

        CompleteFeedback(observers);
    }

    private void CompleteFeedback(List<IObserver<TFeedback>>? observers)
    {
        if (observers is null)
        {
            return;
        }

        // The final admitted dispatch owns channel completion as well as observer completion.
        // Channel continuations and observer callbacks must run outside the subscription gate.
        _feedbackChannel.Writer.TryComplete();

        foreach (var observer in observers)
        {
            observer.OnCompleted();
        }
    }

    async Task<TResult> IActionGoalContext<TResult, TFeedback>.GetResultAsync(CancellationToken cancellationToken)
    {
        using var buffer = await GetResultAsync(cancellationToken).ConfigureAwait(false);
        return (TResult)TResult.CreateFrom(buffer.Data, _textEncoding);
    }

    async Task<TResult> IActionGoalContext<TResult, TFeedback>.GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken)
    {
        using var buffer = await GetResultAsync(timeoutMilliseconds, cancellationToken).ConfigureAwait(false);
        return (TResult)TResult.CreateFrom(buffer.Data, _textEncoding);
    }

    async Task<TResult> IActionGoalContext<TResult, TFeedback>.GetResultAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        using var buffer = await GetResultAsync(timeout, cancellationToken).ConfigureAwait(false);
        return (TResult)TResult.CreateFrom(buffer.Data, _textEncoding);
    }

    async Task<ActionResult<TResult>> IActionGoalContext<TResult, TFeedback>.GetResultWithStatusAsync(CancellationToken cancellationToken)
    {
        return CreateResultWithStatus(await GetResultWithStatusAsync(cancellationToken).ConfigureAwait(false));
    }

    async Task<ActionResult<TResult>> IActionGoalContext<TResult, TFeedback>.GetResultWithStatusAsync(int timeoutMilliseconds, CancellationToken cancellationToken)
    {
        return CreateResultWithStatus(await GetResultWithStatusAsync(timeoutMilliseconds, cancellationToken).ConfigureAwait(false));
    }

    async Task<ActionResult<TResult>> IActionGoalContext<TResult, TFeedback>.GetResultWithStatusAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        return CreateResultWithStatus(await GetResultWithStatusAsync(timeout, cancellationToken).ConfigureAwait(false));
    }

    private ActionResult<TResult> CreateResultWithStatus(ActionResult result)
    {
        if (!result.IsSuccessful)
        {
            return new(result.Status, default);
        }

        using (result.Result)
        {
            return new(result.Status, (TResult)TResult.CreateFrom(result.Result.Data, _textEncoding));
        }
    }

    private class Subscription : IDisposable
    {
        internal static readonly Subscription Empty = new(0, null);
        private readonly int _id;
        private readonly ActionGoalContext<TResult, TFeedback>? _tracker;

        public Subscription(int id, ActionGoalContext<TResult, TFeedback>? tracker)
        {
            _id = id;
            _tracker = tracker;
        }

        public void Dispose()
        {
            _tracker?.Unsubscribe(_id);
        }
    }
}
