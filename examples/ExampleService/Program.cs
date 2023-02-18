using Rcl;
using Rosidl.Messages.Std;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("example_svc");

var resp = new EmptyServiceResponse();

using var defaultSvc = node.CreateService<EmptyService,
    EmptyServiceRequest, EmptyServiceResponse>("/default_svc", (request, obj) =>
    {
        Console.WriteLine("Received request to /default_svc");
        return resp;
    });

using var nativeSvc = node.CreateNativeService<EmptyService>("/native_svc", (request, response, obj) =>
    {
        Console.WriteLine("Received request to /native_svc");
    });

using var concurrentSvc = node.CreateConcurrentService<EmptyService,
    EmptyServiceRequest, EmptyServiceResponse>("/concurrent_svc", (req, obj, ct) =>
    {
        Console.WriteLine("Received request to /concurrent_svc");
        return Task.FromResult(resp);
    });

using var concurrentNativeSvc = node.CreateConcurrentNativeService<EmptyService>(
    "/concurrent_native_svc", (req, resp, state, ct) =>
{
    Console.WriteLine("Received request to /concurrent_native_svc");
    return Task.CompletedTask;
});

Console.ReadLine();