using Rosidl.Runtime;
using System;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Channels;

namespace Rcl.Actions.Client;

internal class ActionGoalContext<TResult, TFeedback> : ActionGoalContextBase, IActionGoalContext<TResult, TFeedback>
    where TFeedback : IActionFeedback
    where TResult : IActionResult
{
    private readonly Channel<TFeedback> _feedbackChannel;
    private readonly ConcurrentDictionary<int, IObserver<TFeedback>> _observers = new();
    private readonly Encoding _textEncoding;

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

    public override bool HasFeedbackListeners => _channelReaders > 0 || !_observers.IsEmpty;

    public override void OnFeedbackReceived(RosMessageBuffer feedback)
    {
        TFeedback msg;
        using (feedback)
        {
            msg = (TFeedback)TFeedback.CreateFrom(feedback.Data, _textEncoding);
        }

        _feedbackChannel.Writer.TryWrite(msg);
        foreach (var (_, obs) in _observers)
        {
            obs.OnNext(msg);
        }
    }

    public async IAsyncEnumerable<TFeedback> ReadFeedbacksAsync([EnumeratorCancellation] CancellationToken cancellationToken)
    {
        Interlocked.Increment(ref _channelReaders);

        try
        {
            await foreach (var buffer in _feedbackChannel.Reader.ReadAllAsync(cancellationToken))
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
        var id = Interlocked.Increment(ref _subscriberId);
        _observers[id] = observer;
        return new Subscription(id, this);
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
        if (_feedbackChannel.Writer.TryComplete())
        {
            foreach (var (_, obs) in _observers)
            {
                obs.OnCompleted();
            }
        }
    }

    async Task<TResult> IActionGoalContext<TResult, TFeedback>.GetResultAsync(CancellationToken cancellationToken)
    {
        using var buffer = await GetResultAsync(cancellationToken);
        return (TResult)TResult.CreateFrom(buffer.Data, _textEncoding);
    }

    async Task<TResult> IActionGoalContext<TResult, TFeedback>.GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken)
    {
        using var buffer = await GetResultAsync(timeoutMilliseconds, cancellationToken);
        return (TResult)TResult.CreateFrom(buffer.Data, _textEncoding);
    }

    async Task<TResult> IActionGoalContext<TResult, TFeedback>.GetResultAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        using var buffer = await GetResultAsync(timeout, cancellationToken);
        return (TResult)TResult.CreateFrom(buffer.Data, _textEncoding);
    }

    async Task<ActionResult<TResult>> IActionGoalContext<TResult, TFeedback>.GetResultWithStatusAsync(CancellationToken cancellationToken)
    {
        return CreateResultWithStatus(await GetResultWithStatusAsync(cancellationToken));
    }

    async Task<ActionResult<TResult>> IActionGoalContext<TResult, TFeedback>.GetResultWithStatusAsync(int timeoutMilliseconds, CancellationToken cancellationToken)
    {
        return CreateResultWithStatus(await GetResultWithStatusAsync(timeoutMilliseconds, cancellationToken));
    }

    async Task<ActionResult<TResult>> IActionGoalContext<TResult, TFeedback>.GetResultWithStatusAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        return CreateResultWithStatus(await GetResultWithStatusAsync(timeout, cancellationToken));
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
        private readonly int _id;
        private readonly ActionGoalContext<TResult, TFeedback> _tracker;

        public Subscription(int id, ActionGoalContext<TResult, TFeedback> tracker)
        {
            _id = id;
            _tracker = tracker;
        }

        public void Dispose()
        {
            _tracker._observers.TryRemove(_id, out _);
        }
    }
}
