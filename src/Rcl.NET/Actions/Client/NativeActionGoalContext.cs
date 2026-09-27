using System.Runtime.CompilerServices;
using System.Threading.Channels;

namespace Rcl.Actions.Client;

internal class NativeActionGoalContext : ActionGoalContextBase, INativeActionGoalContext
{
    private readonly Channel<RosMessageBuffer> _feedbackChannel;

    private readonly object _feedbackGate = new();
    private int _activeDispatches;
    private bool _completed, _disposeRequested;
    private int _channelReaders = 0;

    public NativeActionGoalContext(Guid goalId, IActionClientImpl actionClient)
        : base(goalId, actionClient)
    {
        var opts = new BoundedChannelOptions(actionClient.Options.QueueSize)
        {
            AllowSynchronousContinuations = actionClient.Options.AllowSynchronousContinuations,
            SingleReader = false,
            SingleWriter = true,
            FullMode = actionClient.Options.FullMode
        };
        _feedbackChannel = Channel
            .CreateBounded<RosMessageBuffer>(opts, x => x.Dispose());
    }

    public override bool HasFeedbackListeners => Volatile.Read(ref _channelReaders) > 0;

    public override void OnFeedbackReceived(RosMessageBuffer feedback)
    {
        bool admitted;

        lock (_feedbackGate)
        {
            admitted = !_completed;

            if (admitted)
            {
                _activeDispatches++;
            }
        }

        if (!admitted)
        {
            feedback.Dispose();
            return;
        }

        try
        {
            if (!_feedbackChannel.Writer.TryWrite(feedback))
            {
                feedback.Dispose();
            }
        }
        finally
        {
            bool complete, drain;

            lock (_feedbackGate)
            {
                complete = --_activeDispatches == 0 && _completed;
                drain = _disposeRequested;
            }

            if (complete)
            {
                CompleteFeedback(drain);
            }
        }
    }

    public async IAsyncEnumerable<RosMessageBuffer> ReadFeedbacksAsync([EnumeratorCancellation] CancellationToken cancellationToken)
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

    protected override void OnGoalStateChanged(ActionGoalStatus state)
    {
        if (state is ActionGoalStatus.Canceled or ActionGoalStatus.Succeeded or ActionGoalStatus.Aborted)
        {
            CloseFeedback(dispose: false);
        }
    }

    protected override void OnDispose()
    {
        CloseFeedback(dispose: true);
    }

    private void CloseFeedback(bool dispose)
    {
        bool complete;

        lock (_feedbackGate)
        {
            _completed = true;
            _disposeRequested |= dispose;
            complete = _activeDispatches == 0;
        }

        if (complete)
        {
            CompleteFeedback(dispose);
        }
    }

    private void CompleteFeedback(bool drain)
    {
        // No admitted writer remains. Run continuations and buffer destruction outside the gate.
        _feedbackChannel.Writer.TryComplete();

        if (drain)
        {
            while (_feedbackChannel.Reader.TryRead(out var item))
            {
                item.Dispose();
            }
        }
    }
}
