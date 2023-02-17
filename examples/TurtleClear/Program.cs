using Rcl;
using Rosidl.Messages.Std;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("turtle_clear");
using var client = node.CreateClient<
    EmptyService, EmptyServiceRequest, EmptyServiceResponse>("/clear");

await client.InvokeAsync(new EmptyServiceRequest(), 1000);