using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl;

/// <summary>
/// 
/// </summary>
/// <param name="Arguments"></param>
/// <param name="DomaindId">Domain ID. Only on foxy.</param>
/// <param name="UseGlobalArguments"></param>
/// <param name="EnableRosOut"></param>
public record NodeOptions(string[]? Arguments = null, nuint? DomaindId = null, bool UseGlobalArguments = true, bool EnableRosOut = true)
{
    public static NodeOptions Default { get; } = new();
}