namespace Rcl;

public interface IRclClock: IRclObject
{
    TimeSpan Elapsed { get; }
    DateTimeOffset Now { get; }
}