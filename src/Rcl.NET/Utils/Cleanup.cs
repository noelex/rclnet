using Rcl.SafeHandles;

namespace Rcl.Utils;

internal static class Cleanup
{
    [ThreadStatic]
    private static List<Exception>? s_errors;

    // Shutdown owns synchronous cleanup only; do not flow this scope to user continuations.
    internal readonly struct ErrorScope : IDisposable
    {
        private readonly List<Exception>? _previous;

        internal ErrorScope(List<Exception> errors)
        {
            _previous = s_errors;
            s_errors = errors;
        }

        public void Dispose() => s_errors = _previous;
    }

    internal static void RecordReleaseFailure(string handleType)
    {
        try
        {
            s_errors?.Add(new InvalidOperationException(
                $"{handleType} cleanup failed. See handle release diagnostics for details."));
        }
        catch
        {
            // SafeHandle release must not throw, including while recording its failure.
        }
    }

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
                s_errors?.Add(error);
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
