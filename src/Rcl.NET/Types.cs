using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl;

public record NodeOptions(string[]? Arguments = null, nuint? DomaindId = null, bool UseGlobalArguments = true, bool EnableRosOut = true)
{
    public static NodeOptions Default { get; } = new();
}