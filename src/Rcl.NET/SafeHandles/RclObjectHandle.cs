using Microsoft.Win32.SafeHandles;
using Rcl.Interop;
using System.Runtime.InteropServices;

namespace Rcl.SafeHandles;

internal abstract class RclObjectHandle : SafeHandleZeroOrMinusOneIsInvalid
{
    private int _closing, _releaseRequested;
    private RclObjectHandle? _shutdownDomain;
    private RclObjectHandle? _dependency0, _dependency1;
    private bool _dependenciesSet;
    private InitializationState _initialization;

    // Borrowed handles also need ReleaseHandle to return their dependency refs.
    protected RclObjectHandle() : base(true) { }

    internal bool IsClosing => Volatile.Read(ref _closing) != 0;
    internal bool IsReleaseRequested => Volatile.Read(ref _releaseRequested) != 0;
    protected bool NeedsCleanup => _initialization != InitializationState.Storage;

    internal bool TryBeginClose() => Interlocked.CompareExchange(ref _closing, 1, 0) == 0;
    internal void RequestRelease() => Dispose();

    protected override void Dispose(bool disposing)
    {
        TryBeginClose();
        if (Interlocked.Exchange(ref _releaseRequested, 1) == 0)
            base.Dispose(disposing);
    }

    protected void MarkCleanupRequired() => _initialization = InitializationState.CleanupRequired;
    protected void MarkInitialized() => _initialization = InitializationState.Initialized;
    protected void SetShutdownDomain(RclObjectHandle domain) => _shutdownDomain = domain;

    // Construction only: acquire all refs before checking the immutable ancestor graph.
    protected void SetDependencies(RclObjectHandle parent0, RclObjectHandle? parent1 = null)
    {
        if (IsClosing || _initialization != InitializationState.Storage || _dependenciesSet
            || ReferenceEquals(this, parent0) || ReferenceEquals(this, parent1)
            || ReferenceEquals(parent0, parent1))
            throw new InvalidOperationException("Dependencies must be distinct and assigned once.");

        bool added0 = false, added1 = false;
        try
        {
            parent0.DangerousAddRef(ref added0);
            parent1?.DangerousAddRef(ref added1);
            var domain = _shutdownDomain ?? parent0._shutdownDomain ?? parent1?._shutdownDomain;
            if ((parent0._shutdownDomain != null && parent0._shutdownDomain != domain)
                || (parent1?._shutdownDomain != null && parent1._shutdownDomain != domain))
                throw new InvalidOperationException("Dependencies belong to different shutdown domains.");

            parent0.ThrowIfDescendantClosed();
            parent1?.ThrowIfDescendantClosed();
            _dependency0 = parent0;
            _dependency1 = parent1;
            _shutdownDomain = domain;
            _dependenciesSet = true;
            added0 = added1 = false;
        }
        finally
        {
            if (added1) ReturnDependency(parent1);
            if (added0) ReturnDependency(parent0);
        }
    }

    internal void ThrowIfOperationClosed()
    {
        if (IsClosing || IsInvalid || _initialization != InitializationState.Initialized
            || (_shutdownDomain?.IsClosing ?? false))
            throw new ObjectDisposedException(GetType().Name);
    }

    private void ThrowIfDescendantClosed()
    {
        ThrowIfOperationClosed();
        _dependency0?.ThrowIfDescendantClosed();
        _dependency1?.ThrowIfDescendantClosed();
    }

    protected bool ReleaseDependencies()
    {
        var parent1 = _dependency1;
        var parent0 = _dependency0;
        _dependency1 = _dependency0 = null;
        bool success = ReturnDependency(parent1);
        return ReturnDependency(parent0) && success;
    }

    private bool ReturnDependency(RclObjectHandle? parent)
    {
        try { parent?.DangerousRelease(); return true; }
        catch (Exception error) { ReportReleaseException("dependency release", error); return false; }
    }

    protected bool CheckReleaseResult(rcl_ret_t result, string api)
    {
        if (result == 0) return true;
        try
        {
            string? message = null;
            try
            {
                if (rcutils_error_is_set())
                {
                    try
                    {
                        unsafe
                        {
                            var error = rcutils_get_error_string();
                            message = Marshal.PtrToStringUTF8((IntPtr)error.str);
                        }
                    }
                    finally { rcutils_reset_error(); }
                }
            }
            catch (Exception error) { message = error.ToString(); }
            WriteReleaseError(new(GetType().Name, api, (int)result, message));
        }
        catch { /* Diagnostics must never interrupt cleanup. */ }
        return false;
    }

    protected void ReportReleaseException(string phase, Exception error)
    {
        try { WriteReleaseError(new(GetType().Name, phase, null, error.ToString())); }
        catch { /* Includes failures while formatting or recording the error. */ }
    }

    protected virtual void WriteReleaseError(HandleReleaseError error) => HandleReleaseDiagnostics.Record(error);
    private enum InitializationState { Storage, CleanupRequired, Initialized }
}

internal unsafe abstract class RclObjectHandle<T> : RclObjectHandle where T : unmanaged
{
    private readonly bool _ownsStorage;

    protected RclObjectHandle()
    {
        _ownsStorage = true;
        SetHandle(Marshal.AllocHGlobal(sizeof(T)));
        *(T*)handle = default;
    }

    protected RclObjectHandle(IntPtr handle, RclObjectHandle? owner = null)
    {
        SetHandle(handle);
        try
        {
            ThrowIfInvalidOrClosed();
            if (owner != null) SetDependencies(owner);
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    internal RclHandleLease<T> Acquire() => new(this);

    public void ThrowIfInvalidOrClosed()
    {
        if (IsInvalid || IsClosed || IsClosing)
            throw new ObjectDisposedException(GetType().Name);
    }

    // Legacy call sites and constructors migrate separately to Acquire/dependencies.
    public T* Object
    {
        get { ThrowIfInvalidOrClosed(); return (T*)handle; }
    }

    internal T* DangerousObject => (T*)handle;
    protected abstract bool ReleaseHandleCore(T* ptr);

    protected override bool ReleaseHandle()
    {
        bool success = true;
        try
        {
            if (_ownsStorage && NeedsCleanup)
                success = ReleaseHandleCore((T*)handle);
        }
        catch (Exception error)
        {
            success = false;
            ReportReleaseException("native cleanup", error);
        }

        try
        {
            if (_ownsStorage) Marshal.FreeHGlobal(handle);
        }
        catch (Exception error)
        {
            success = false;
            ReportReleaseException("storage free", error);
        }
        finally { SetHandle(IntPtr.Zero); }

        // Derived native-state locks have been left before a parent can release.
        return ReleaseDependencies() && success;
    }
}

internal unsafe ref struct RclHandleLease<T> where T : unmanaged
{
    private RclObjectHandle<T>? _owner;
    private T* _object;

    internal RclHandleLease(RclObjectHandle<T> owner)
    {
        _owner = null;
        _object = null;
        bool added = false;
        try
        {
            owner.DangerousAddRef(ref added);
            owner.ThrowIfOperationClosed();
            _object = owner.DangerousObject;
            _owner = owner;
        }
        catch
        {
            if (added) owner.DangerousRelease();
            throw;
        }
    }

    public readonly T* Object => _owner is null
        ? throw new ObjectDisposedException(nameof(RclHandleLease<T>)) : _object;

    // Keep leases local: copying a ref struct also copies its reference ownership.
    public void Dispose()
    {
        var owner = _owner;
        _owner = null;
        _object = null;
        owner?.DangerousRelease();
    }
}
