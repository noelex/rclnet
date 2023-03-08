using Rcl.Logging;
using Rcl.Parameters;
using Rcl.Qos;
using Rosidl.Messages.Rosgraph;
using System.Xml.Linq;

namespace Rcl.Internal.NodeServices;

class ExternalTimeSource : IDisposable
{
    private const string UseSimTime = "use_sim_time";

    private readonly IRclNode _node;
    private readonly QosProfile _qos;
    private readonly IDisposable _reg;

    private bool _overrideEnabled;
    private IRclNativeSubscription? _subscription;

    public ExternalTimeSource(IRclNode node, QosProfile clockQoS)
    {
        _node = node;
        _qos = clockQoS;

        _reg = node.Parameters.RegisterParameterChangingEvent(OnParameterChanging, this);
        node.Parameters.Declare(UseSimTime, false);
    }

    private static ValidationResult OnParameterChanging(ReadOnlySpan<ParameterChangingInfo> info, object? state)
    {
        var self = (ExternalTimeSource)state!;
        foreach (var (descriptor, oldValue, newValue) in info)
        {
            if (descriptor.Name != UseSimTime)
            {
                continue;
            }

            if (oldValue == newValue)
            {
                continue;
            }

            if (newValue.AsBoolean())
            {
                if(self._node.Clock.Type != RclClockType.Ros)
                {
                    return ValidationResult.Failure("use_sim_time parameter can't be true while not using ROS clock.");
                }

                self._overrideEnabled = self._node.Clock.Impl.IsRosTimeOverrideEnabled;

                // Suppress asynchronous scheduling because clock updates may be published very frequently.
                self._subscription = self._node.CreateNativeSubscription<Clock>("/clock", 
                    new(qos: self._qos, allowSynchronousContinuations: true));
                _ = self.UpdateClockAsync(self._subscription);

                self._node.Context.DefaultLogger.LogDebug("use_sim_time is enabled.");
            }
            else
            {
                self._subscription?.Dispose();
                if (self._overrideEnabled)
                {
                    self._node.Clock.Impl.ToggleRosTimeOverride(false);
                    self._overrideEnabled = false;
                }

                self._node.Context.DefaultLogger.LogDebug("use_sim_time is disabled.");
            }
        }

        return ValidationResult.Success();
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
        _reg.Dispose();
        _node.Parameters.Undeclare(UseSimTime);

        _subscription?.Dispose();
        if (_overrideEnabled)
        {
            _node.Clock.Impl.ToggleRosTimeOverride(false);
            _overrideEnabled = false;
        }
    }
}
