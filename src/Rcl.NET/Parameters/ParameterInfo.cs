using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

public record struct ParameterChangingInfo(ParameterDescriptor Descriptor, Variant OldValue, Variant NewValue);
