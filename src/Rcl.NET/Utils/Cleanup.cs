using Rcl.SafeHandles;

namespace Rcl.Utils;

internal static class Cleanup
{
    internal static void Run(Action<object?> callback, object? state)
    {
        try
        {
            callback(state);
        }
        catch (Exception error)
        {
            try
            {
                HandleReleaseDiagnostics.Record(new(nameof(Cleanup), "managed cleanup", null, error.ToString()));
            }
            catch
            {
                // Diagnostics cannot interrupt remaining cleanup.
            }
        }
    }

    internal static void Dispose(IDisposable? resource)
    {
        if (resource != null)
        {
            Run(static state => ((IDisposable)state!).Dispose(), resource);
        }
    }
}
