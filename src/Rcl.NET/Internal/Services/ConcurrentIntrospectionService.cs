using Rcl.Interop;
using Rcl.SafeHandles;
using Rosidl.Runtime;

namespace Rcl.Internal.Services;

// Each asynchronous dispatch owns its buffers and one callback reference until it exits.
internal class ConcurrentIntrospectionService : IntrospectionServiceBase
{
    private readonly IConcurrentNativeServiceHandler _handler;
    private readonly CancellationTokenSource _shutdownSignal = new();
    private readonly object _callbackGate = new();
    private int _inFlight;
    private bool _stopping, _cancelComplete, _detached;

    public unsafe ConcurrentIntrospectionService(
        RclNodeImpl node, string serviceName, IConcurrentNativeServiceHandler handler,
        TypeSupportHandle typesupport, ServerOptions options)
        : base(node, serviceName, typesupport, options)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override unsafe void DispatchRequest(RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t id)
    {
        CancellationToken token;

        lock (_callbackGate)
        {
            if (_stopping)
            {
                request.Dispose();
                response.Dispose();
                return;
            }

            token = _shutdownSignal.Token;
            _inFlight++;
        }

        _ = DispatchAsync(request, response, id, token);
    }

    private async Task DispatchAsync(RosMessageBuffer request, RosMessageBuffer response,
        rmw_request_id_t id, CancellationToken token)
    {
        try
        {
            using (request)
            {
                using (response)
                {
                    await _handler.ProcessRequestAsync(request, response, token).ConfigureAwait(false);
                    token.ThrowIfCancellationRequested();
                    RclException.ThrowIfNonSuccess(SendResponse(id, response.Data));
                }
            }
        }
        catch (OperationCanceledException) when (token.IsCancellationRequested)
        {
        }
        catch (ObjectDisposedException) when (Handle.IsClosing || Context.Handle.IsClosing)
        {
        }
        catch (Exception error)
        {
            HandleReleaseDiagnostics.Record(new(GetType().Name, "asynchronous service callback", null, error.ToString()));
        }
        finally
        {
            bool release;

            lock (_callbackGate)
            {
                release = --_inFlight == 0 && _cancelComplete && _detached;
            }

            if (release)
            {
                _shutdownSignal.Dispose();
            }
        }
    }

    protected override void OnStopped()
    {
        lock (_callbackGate)
        {
            _stopping = true;
        }

        try
        {
            _shutdownSignal.Cancel();
        }
        finally
        {
            bool release;

            lock (_callbackGate)
            {
                _cancelComplete = true;
                release = _inFlight == 0 && _detached;
            }

            if (release)
            {
                _shutdownSignal.Dispose();
            }
        }
    }

    protected override void OnDetached()
    {
        bool release;

        lock (_callbackGate)
        {
            _detached = true;
            release = _cancelComplete && _inFlight == 0;
        }

        if (release)
        {
            _shutdownSignal.Dispose();
        }
    }
}
