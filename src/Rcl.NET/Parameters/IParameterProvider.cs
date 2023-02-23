using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

public interface IParameterProvider
{
    Variant Declare(ParameterDescriptor descriptor, Variant defaultValue, bool ignoreOverride = false);

    Variant Declare(string name, Variant defaultValue, bool ignoreOverride = false);

    Variant Declare(string name, ValueType type, bool ignoreOverride = false);

    Variant Declare(ParameterDescriptor descriptor, bool ignoreOverride = false);

    Parameter Get(string name);

    bool TryGet(string name, out Parameter parameter);

    void Set(Parameter parameter);

    Variant GetValue(string name);

    bool TryGetValue(string name, out Variant variant);

    void Set(string name, Variant value);

    void Undeclare(string name);

    bool IsDeclared(string name);
}
