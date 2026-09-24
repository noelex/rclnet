namespace Rcl.SafeHandles;

internal sealed record HandleReleaseError(string HandleType, string Api, int? ReturnCode, string? Message);

internal static class HandleReleaseDiagnostics
{
    // Bounded process-local history, independent of ROS logging and user callbacks.
    private static readonly HandleReleaseError?[] s_errors = new HandleReleaseError[64];
    private static int s_next;

    internal static void Record(HandleReleaseError error)
    {
        uint index = unchecked((uint)Interlocked.Increment(ref s_next) - 1);
        Volatile.Write(ref s_errors[index % (uint)s_errors.Length], error);
    }

    internal static HandleReleaseError?[] Snapshot() => s_errors.ToArray();
}
