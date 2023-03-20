using System.Runtime.CompilerServices;
using System.Threading.Channels;

namespace Rcl.Actions.Client;

internal class NativeActionGoalContext : ActionGoalContextBase, INativeActionGoalContext
{
    private readonly Channel<RosMessageBuffer> _feedbackChannel;

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

    public override bool HasFeedbackListeners => _channelReaders > 0;

    public override void OnFeedbackReceived(RosMessageBuffer feedback)
    {
        // Should never return false as FullMode cannot be set to Wait, but just in case.
        if (!_feedbackChannel.Writer.TryWrite(feedback))
        {
            feedback.Dispose();
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
            _feedbackChannel.Writer.TryComplete();
        }
    }

    protected override void OnDispose()
    {
        _feedbackChannel.Writer.TryComplete();

        // Drain the channel to make sure all pending RosMessageBuffers are disposed.
        while (_feedbackChannel.Reader.TryRead(out var item))
        {
            item.Dispose();
        }
    }
}
