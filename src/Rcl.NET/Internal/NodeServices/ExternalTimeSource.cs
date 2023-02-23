using Rcl.Parameters;
using Rcl.Qos;
using Rosidl.Messages.Rosgraph;

namespace Rcl.Internal.NodeServices;

class ExternalTimeSource : IDisposable
{
    private const string UseSimTime = "use_sim_time";

    private readonly IRclNode _node;
    private readonly IRclNativeSubscription? _subscription;

    private bool _overrideEnabled;

    public ExternalTimeSource(IRclNode node, QosProfile clockQoS)
    {
        _node = node;

        if (!node.Parameters.TryGetValue(UseSimTime, out var useSimTime))
        {
            useSimTime = node.Parameters.Declare(UseSimTime, false);
        }

        if (useSimTime.Kind != ValueKind.Bool)
        {
            throw new RclException($"Invalid type '{useSimTime.Kind}' for parameter '{UseSimTime}', should be 'bool'");
        }

        if (useSimTime.AsBoolean())
        {
            if (node.Clock.Type != RclClockType.Ros)
            {
                throw new RclException("use_sim_time parameter can't be true while not using ROS clock.");
            }

            _overrideEnabled = node.Clock.Impl.IsRosTimeOverrideEnabled;

            _subscription = node.CreateNativeSubscription<Clock>("/clock", clockQoS);
            _ = UpdateClockAsync(_subscription);
        }
    }

    private async Task UpdateClockAsync(IRclNativeSubscription sub)
    {
        await foreach (var e in sub.ReadAllAsync())
        {
            using (e) UpdateClock(e);
        }
    }

    private void UpdateClock(RosMessageBuffer buffer)
    {
        ref var clock = ref buffer.AsRef<Clock.Priv>();
        if (!_overrideEnabled)
        {
            _node.Clock.Impl.ToggleRosTimeOverride(true);
            _overrideEnabled = true;
        }

        var t = clock.Clock_.Sec * 1000_000_000L + clock.Clock_.Nanosec;
        _node.Clock.Impl.SetRosTimeOverride(t);
    }

    public void Dispose()
    {
        _subscription?.Dispose();
        if (_overrideEnabled)
        {
            _node.Clock.Impl.ToggleRosTimeOverride(false);
        }
    }
}
