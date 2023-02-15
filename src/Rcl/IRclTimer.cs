namespace Rcl;

public interface IRclTimer:IRclWaitObject
{
    bool IsPaused { get; }

    void Pause();
    void Resume();
}