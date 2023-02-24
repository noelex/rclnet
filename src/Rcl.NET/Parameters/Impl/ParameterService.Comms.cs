using Rcl.Interop;
using Rosidl.Messages.Rcl;

namespace Rcl.Parameters.Impl;
partial class ParameterService :
    IServiceHandler<GetParametersServiceRequest, GetParametersServiceResponse>,
    IServiceHandler<GetParameterTypesServiceRequest, GetParameterTypesServiceResponse>,
    IServiceHandler<ListParametersServiceRequest, ListParametersServiceResponse>,
    IServiceHandler<SetParametersServiceRequest, SetParametersServiceResponse>,
    IServiceHandler<SetParametersAtomicallyServiceRequest, SetParametersAtomicallyServiceResponse>,
    IServiceHandler<DescribeParametersServiceRequest, DescribeParametersServiceResponse>,
    IServiceHandler<DescribeParametersServiceRequestFoxy, DescribeParametersServiceResponseFoxy>
{
    private readonly IRclService
        _getParametersService,
        _getParameterTypesService,
        _listParametersService,
        _setParametersService,
        _setParametersAtomicallyService,
        _describeParametersService;

    private readonly IRclPublisher<ParameterEvent> _parameterEvents;

    private void ShutdownComms()
    {
        _parameterEvents.Dispose();
        _getParametersService.Dispose();
        _getParameterTypesService.Dispose();
        _listParametersService.Dispose();
        _setParametersService.Dispose();
        _setParametersAtomicallyService.Dispose();
        _describeParametersService.Dispose();
    }

    private static ParameterValue ToParameterValue(Variant value)
    {
        var result = new ParameterValue((byte)value.Kind);
        switch (value.Kind)
        {
            case ValueKind.Bool:
                result.BoolValue = (bool)value;
                break;
            case ValueKind.BoolArray:
                result.BoolArrayValue = (bool[])value;
                break;
            case ValueKind.ByteArray:
                result.ByteArrayValue = (byte[])value;
                break;
            case ValueKind.Double:
                result.DoubleValue = (double)value;
                break;
            case ValueKind.DoubleArray:
                result.DoubleArrayValue = (double[])value;
                break;
            case ValueKind.Integer:
                result.IntegerValue = (long)value;
                break;
            case ValueKind.IntegerArray:
                result.IntegerArrayValue = (long[])value;
                break;
            case ValueKind.String:
                result.StringValue = (string)value;
                break;
            case ValueKind.StringArray:
                result.StringArrayValue = (string[])value;
                break;
        }
        return result;
    }

    private static Variant ToVariant(ParameterValue value)
    {
        return (ValueKind)value.Type switch
        {
            ValueKind.Bool => (Variant)value.BoolValue,
            ValueKind.BoolArray => (Variant)value.BoolArrayValue,
            ValueKind.ByteArray => (Variant)value.ByteArrayValue,
            ValueKind.Double => (Variant)value.DoubleValue,
            ValueKind.DoubleArray => (Variant)value.DoubleArrayValue,
            ValueKind.Integer => (Variant)value.IntegerValue,
            ValueKind.IntegerArray => (Variant)value.IntegerArrayValue,
            ValueKind.String => (Variant)value.StringValue,
            ValueKind.StringArray => (Variant)value.StringArrayValue,
            _ => default,
        };
    }

    private static SetParametersResult ConvertResult(ValidationResult result)
    {
        return new(result.IsSuccessful, result.Message);
    }

    private static Parameter ToParameter(ParameterStore ps)
    {
        return new Parameter(ps.Descriptor.Name, ToParameterValue(ps.Value));
    }

    private static Rosidl.Messages.Rcl.ParameterDescriptor ToParameterDescriptor(ParameterDescriptor descriptor)
    {
        return new Rosidl.Messages.Rcl.ParameterDescriptor(
            name: descriptor.Name,
            type: (byte)descriptor.Type,
            description: descriptor.Description,
            additionalConstraints: descriptor.AdditionalConstraints,
            readOnly: descriptor.ReadOnly,
            dynamicTyping: descriptor.DynamicTyping,
            floatingPointRange: descriptor.FloatingPointRange == null ? null :
                new FloatingPointRange[] { new(descriptor.FloatingPointRange.Min, descriptor.FloatingPointRange.Max, descriptor.FloatingPointRange.Step) },
            integerRange: descriptor.IntegerRange == null ? null :
                new IntegerRange[] { new(descriptor.IntegerRange.Min, descriptor.IntegerRange.Max, (ulong)descriptor.IntegerRange.Step) }
            );
    }

    private static ParameterDescriptorFoxy ToParameterDescriptorFoxy(ParameterDescriptor descriptor)
    {
        return new ParameterDescriptorFoxy(
            name: descriptor.Name,
            type: (byte)descriptor.Type,
            description: descriptor.Description,
            additionalConstraints: descriptor.AdditionalConstraints,
            readOnly: descriptor.ReadOnly,
            floatingPointRange: descriptor.FloatingPointRange == null ? null :
                new FloatingPointRange[] { new(descriptor.FloatingPointRange.Min, descriptor.FloatingPointRange.Max, descriptor.FloatingPointRange.Step) },
            integerRange: descriptor.IntegerRange == null ? null :
                new IntegerRange[] { new(descriptor.IntegerRange.Min, descriptor.IntegerRange.Max, (ulong)descriptor.IntegerRange.Step) }
            );
    }

    private void PublishRemovedParameter(ParameterStore ps)
    {
        var t = _node.Clock.Elapsed.ToRmwTime();
        _parameterEvents.Publish(new ParameterEvent(
            stamp: new(sec: (int)t.sec, nanosec: (uint)t.nsec),
            node: _node.FullyQualifiedName,
            deletedParameters: new[] { ToParameter(ps) }
        ));
    }

    private void PublishNewParameter(ParameterStore ps)
    {
        var t = _node.Clock.Elapsed.ToRmwTime();
        _parameterEvents.Publish(new ParameterEvent(
            stamp: new(sec: (int)t.sec, nanosec: (uint)t.nsec),
            node: _node.FullyQualifiedName,
            changedParameters: new[] { ToParameter(ps) }
        ));
    }

    private void PublishChangedParameters(IEnumerable<ParameterStore> ps)
    {
        var t = _node.Clock.Elapsed.ToRmwTime();
        _parameterEvents.Publish(new ParameterEvent(
            stamp: new(sec: (int)t.sec, nanosec: (uint)t.nsec),
            node: _node.FullyQualifiedName,
            changedParameters: ps.Select(ToParameter).ToArray()
        ));
    }

    public GetParametersServiceResponse ProcessRequest(GetParametersServiceRequest request)
    {
        var results = new ParameterValue[request.Names.Length];
        for (var i = 0; i < request.Names.Length; i++)
        {
            TryGet(request.Names[i], out var value);
            results[i] = ToParameterValue(value);
        }

        return new GetParametersServiceResponse(results);
    }

    public GetParameterTypesServiceResponse ProcessRequest(GetParameterTypesServiceRequest request)
    {
        var results = new byte[request.Names.Length];
        for (var i = 0; i < request.Names.Length; i++)
        {
            if (TryGetStore(request.Names[i], out var ps).IsSuccessful)
            {
                results[i] = (byte)ps.Descriptor.Type;
            }
        }

        return new(results);
    }

    public ListParametersServiceResponse ProcessRequest(ListParametersServiceRequest request)
    {
        var prefixes = request.Prefixes;
        var depth = request.Depth;

        var matchedNames = new List<string>();
        var matchedPrefixes = new List<string>();

        const char separator = '.';
        foreach (var (k, v) in _parameters)
        {
            var getAll = prefixes.Length == 0 ||
                depth == ListParametersServiceRequest.DEPTH_RECURSIVE ||
                k.Count(x => x == separator) < (int)depth;
            var prefixMatches = prefixes.Any(prefix =>
            {
                if (k == prefix)
                {
                    return true;
                }
                else if (k.Contains(prefix + separator))
                {
                    var substr = k.Substring(prefix.Length);
                    return (depth == ListParametersServiceRequest.DEPTH_RECURSIVE) ||
                    substr.Count(x => x == separator) < (int)depth;
                }
                return false;
            });

            if (getAll || prefixMatches)
            {
                matchedNames.Add(k);
                var lastSeparator = k.LastIndexOf(separator);
                if (lastSeparator >= 0)
                {
                    var prefix = k.Substring(0, lastSeparator);
                    if (!matchedPrefixes.Contains(prefix))
                    {
                        matchedPrefixes.Add(prefix);
                    }
                }
            }
        }

        return new(new(matchedNames.ToArray(), matchedPrefixes.ToArray()));
    }

    public SetParametersServiceResponse ProcessRequest(SetParametersServiceRequest request)
    {
        var results = new SetParametersResult[request.Parameters.Length];
        for (var i = 0; i < results.Length; i++)
        {
            results[i] = ConvertResult(
                SetAndNotify(request.Parameters[i].Name,
                    ToVariant(request.Parameters[i].Value)));
        }
        return new(results);
    }

    public SetParametersAtomicallyServiceResponse ProcessRequest(SetParametersAtomicallyServiceRequest request)
    {
        var parameters = request.Parameters.ToDictionary(x => x.Name, x => ToVariant(x.Value));
        return new(ConvertResult(SetAndNotify(parameters)));
    }

    public DescribeParametersServiceResponse ProcessRequest(DescribeParametersServiceRequest request)
    {
        var results = new Rosidl.Messages.Rcl.ParameterDescriptor[request.Names.Length];
        for (var i = 0; i < results.Length; i++)
        {
            if (TryGetStore(request.Names[i], out var ps).IsSuccessful)
            {
                results[i] = ToParameterDescriptor(ps.Descriptor);
            }
            else
            {
                results[i] = new();
            }
        }
        return new DescribeParametersServiceResponse(results);
    }

    public DescribeParametersServiceResponseFoxy ProcessRequest(DescribeParametersServiceRequestFoxy request)
    {
        var results = new Rosidl.Messages.Rcl.ParameterDescriptorFoxy[request.Names.Length];
        for (var i = 0; i < results.Length; i++)
        {
            if (TryGetStore(request.Names[i], out var ps).IsSuccessful)
            {
                results[i] = ToParameterDescriptorFoxy(ps.Descriptor);
            }
            else
            {
                results[i] = new();
            }
        }
        return new DescribeParametersServiceResponseFoxy(results);
    }
}
