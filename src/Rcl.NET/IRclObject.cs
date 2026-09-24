namespace Rcl;

/// <summary>
/// Represents an RCL object.
/// </summary>
/// <remarks>
/// Disposal closes the object to new native operations. Operations already admitted,
/// native dependents and active wait registrations can defer physical native release.
/// Disposing a node or clock does not dispose independently owned children, but prevents
/// creation of new descendants. Closing their context also rejects new native operations
/// on retained children. Cached managed properties may remain readable after disposal.
/// </remarks>
public interface IRclObject : IDisposable
{
}
