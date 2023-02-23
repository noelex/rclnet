using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

public record struct Parameter(ParameterDescriptor Descriptor, Variant Value)
{
    public static readonly Parameter Empty = new();
}