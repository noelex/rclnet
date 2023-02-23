using Rcl.Qos;
using Rcl.Runtime;

namespace Rcl;

public record NodeOptions
{
    public NodeOptions(
        string[]? arguments = null,
        [SupportedDistribution(RosEnvironment.Foxy)]
        nuint? domaindId = null,
        bool useGlobalArguments = true,
        bool enableRosOut = true,
        RclClock? clock = null,
        QosProfile? clockQos = null)
    {
        Arguments = arguments ?? Array.Empty<string>();
        DomaindId = domaindId;
        UseGlobalArguments = useGlobalArguments;
        EnableRosOut = enableRosOut;
        Clock = clock ?? RclClock.Ros;
        ClockQoS = clockQos ?? QosProfile.Clock;
    }

    public string[] Arguments { get; }

    [SupportedDistribution(RosEnvironment.Foxy)]
    public nuint? DomaindId { get; }

    public bool UseGlobalArguments { get; }

    public bool EnableRosOut { get; }

    public RclClock Clock { get; }

    public QosProfile ClockQoS { get; }

    public static NodeOptions Default { get; } = new();
}