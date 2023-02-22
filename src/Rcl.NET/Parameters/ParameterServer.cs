using Rosidl.Messages.Rcl;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

internal class ParameterServer: IServiceHandler<DescribeParametersServiceRequest, DescribeParametersServiceResponse>
{
    public ParameterServer(RclNodeImpl node)
    {
        node.CreateService<
            DescribeParametersService,
            DescribeParametersServiceRequest,
            DescribeParametersServiceResponse>($"{node.Name}/describe_parameters", this);
    }

    public DescribeParametersServiceResponse ProcessRequest(DescribeParametersServiceRequest request)
    {
        return new();
    }
}
