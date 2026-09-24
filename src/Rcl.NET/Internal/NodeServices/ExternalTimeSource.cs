using Rcl.Logging;
using Rcl.Parameters;
using Rcl.Qos;
using Rosidl.Messages.Rosgraph;
using Rosidl.Runtime;

namespace Rcl.Internal.NodeServices;

class ExternalTimeSource : IDisposable
{
    private const string UseSimTime = "use_sim_time";

    private readonly RclNodeImpl _node;
    private readonly QosProfile _qos;
    private readonly IDisposable _reg;

    private readonly object _gate = new();
    private bool _disposed;
    private bool _overrideEnabled;
    private IRclNativeSubscription? _subscription;

    public ExternalTimeSource(RclNodeImpl node, QosProfile clockQoS)
    {
        _node = node;
        _qos = clockQoS;

        _reg = node.Parameters.RegisterParameterChangingEvent(OnParameterChanging, this);

        try
        {
            node.Parameters.Declare(UseSimTime, false);
        }
        catch
        {
            _reg.Dispose();
            _subscription?.Dispose();
            throw;
        }
    }

    private static ValidationResult OnParameterChanging(ReadOnlySpan<ParameterChangingInfo> info, object? state)
    {
        var self = (ExternalTimeSource)state!;

        lock (self._gate)
        {
            if (self._disposed)
            {
                return ValidationResult.Failure("Time source is disposed.");
            }

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
                    if (self._node.Clock.Type != RclClockType.Ros)
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
    }

    private async Task UpdateClockAsync(IRclNativeSubscription sub)
    {
        await foreach (var e in sub.ReadAllAsync().ConfigureAwait(false))
        {
            using (e)
            {
                UpdateClock(e);
            }
        }
    }

    private void UpdateClock(RosMessageBuffer buffer)
    {
        if (!_overrideEnabled)
        {
            _node.Clock.Impl.ToggleRosTimeOverride(true);
            _overrideEnabled = true;
        }

        long t;

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var clock = ref buffer.AsRef<Clock.Priv>();
            t = clock.Clock_.Sec * 1000_000_000L + clock.Clock_.Nanosec;
        }
        else
        {
            ref var clock = ref buffer.AsRef<Clock.PrivV2>();
            t = clock.Clock_.Sec * 1000_000_000L + clock.Clock_.Nanosec;
        }

        _node.Clock.Impl.SetRosTimeOverride(t);
    }

    public void Dispose()
    {
        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;
            _reg.Dispose();

            try
            {
                _node.Parameters.Undeclare(UseSimTime);
            }
            catch (ObjectDisposedException) when (_node.Context.Handle.IsClosing)
            {
                // Undeclare already removed the parameter; a closed domain cannot publish its event.
            }
            finally
            {
                _subscription?.Dispose();

                if (_overrideEnabled)
                {
                    _node.Clock.Impl.ToggleRosTimeOverride(false);
                    _overrideEnabled = false;
                }
            }
        }
    }
}
