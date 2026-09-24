using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeContextHandle : RclObjectHandle<rcl_context_t>
{
    internal static readonly object LoggingGate = new();
    private static int s_loggingReferences;
    private bool _ownsLogging;
    private bool _shutdownRequested;
    private bool _shutdownSucceeded = true;

    internal object LifecycleGate { get; } = new();
    internal static int LoggingReferences { get { lock (LoggingGate) return s_loggingReferences; } }

    public SafeContextHandle(string[] args)
    {
        SetShutdownDomain(this);
        rcl_init_options_t opts = default;
        bool optionsInitialized = false;
        try
        {
            opts = rcl_get_zero_initialized_init_options();
            *Object = rcl_get_zero_initialized_context();
            RclException.ThrowIfNonSuccess(rcl_init_options_init(&opts, RclAllocator.Default.Object));
            optionsInitialized = true;
            int argc = args.Length;
            if (argc > 0)
            {
                var bufferSize = InteropHelpers.GetUtf8BufferSize(args);
                Span<byte> argBuffer = stackalloc byte[bufferSize];
                var argv = stackalloc byte*[argc];
                InteropHelpers.FillUtf8Buffer(args, argBuffer, argv);
                RclException.ThrowIfNonSuccess(rcl_init(argc, argv, &opts, Object));
            }
            else
            {
                RclException.ThrowIfNonSuccess(rcl_init(0, null, &opts, Object));
            }
            MarkInitialized();
            lock (LoggingGate)
            {
                if (s_loggingReferences == 0)
                {
                    var allocator = RclAllocator.Default.Object;
                    try
                    {
                        RclException.ThrowIfNonSuccess(rcl_logging_configure(&Object->global_arguments, &allocator));
                    }
                    catch
                    {
                        // Failed configure can leave rosout/external logging partially initialized.
                        try { CheckReleaseResult(rcl_logging_fini(), nameof(rcl_logging_fini)); }
                        catch (Exception error) { ReportReleaseException(nameof(rcl_logging_fini), error); }
                        throw;
                    }
                }
                s_loggingReferences++;
                _ownsLogging = true;
            }
        }
        catch
        {
            Dispose();
            throw;
        }
        finally
        {
            if (optionsInitialized)
            {
                try { CheckReleaseResult(rcl_init_options_fini(&opts), nameof(rcl_init_options_fini)); }
                catch (Exception error) { ReportReleaseException(nameof(rcl_init_options_fini), error); }
            }
        }
    }

    internal override bool TryBeginClose()
    {
        lock (LifecycleGate) return base.TryBeginClose();
    }

    // The event loop retains the owner ref while requesting shutdown. Children may
    // outlive that loop, so context fini and logging release happen only at last ref.
    internal bool Shutdown()
    {
        lock (LifecycleGate)
        {
            TryBeginClose();
            if (_shutdownRequested) return _shutdownSucceeded;
            _shutdownRequested = true;
            try { _shutdownSucceeded = CheckReleaseResult(rcl_shutdown(DangerousObject), nameof(rcl_shutdown)); }
            catch (Exception error)
            {
                _shutdownSucceeded = false;
                ReportReleaseException(nameof(rcl_shutdown), error);
            }
            return _shutdownSucceeded;
        }
    }

    protected override bool ReleaseHandleCore(rcl_context_t* ptr)
    {
        bool success = Shutdown();
        try { return CheckReleaseResult(rcl_context_fini(ptr), nameof(rcl_context_fini)) && success; }
        catch (Exception error)
        {
            ReportReleaseException(nameof(rcl_context_fini), error);
            return false;
        }
    }

    protected override bool ReleaseAdditionalResources()
    {
        lock (LoggingGate)
        {
            if (!_ownsLogging) return true;
            _ownsLogging = false;
            return --s_loggingReferences != 0 || CheckReleaseResult(rcl_logging_fini(), nameof(rcl_logging_fini));
        }
    }
}
