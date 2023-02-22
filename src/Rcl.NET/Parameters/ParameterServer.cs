using Rcl.Interop;
using Rcl.SafeHandles;
using Rosidl.Messages.Rcl;
using System;
using System.Collections.Concurrent;

namespace Rcl.Parameters;

internal class ParameterServer : IServiceHandler<DescribeParametersServiceRequest, DescribeParametersServiceResponse>
{
    private static readonly Dictionary<Type, ParameterType> _clrToRosMapping = new()
    {
        [typeof(bool)] = ParameterType.Bool,

        [typeof(byte)] = ParameterType.Integer,
        [typeof(sbyte)] = ParameterType.Integer,
        [typeof(short)] = ParameterType.Integer,
        [typeof(ushort)] = ParameterType.Integer,
        [typeof(int)] = ParameterType.Integer,
        [typeof(uint)] = ParameterType.Integer,
        [typeof(long)] = ParameterType.Integer,
        [typeof(ulong)] = ParameterType.Integer,

        [typeof(float)] = ParameterType.Double,
        [typeof(double)] = ParameterType.Double,

        [typeof(string)] = ParameterType.String,

        [typeof(bool[])] = ParameterType.BoolArray,
        [typeof(byte[])] = ParameterType.ByteArray,
        [typeof(long[])] = ParameterType.IntegerArray,
        [typeof(double[])] = ParameterType.DoubleArray,
        [typeof(string[])] = ParameterType.StringArray,
    };

    private readonly ConcurrentDictionary<string, Parameter> _parameters = new();

    public unsafe ParameterServer(RclNodeImpl node, ParameterDictionary paramOverrides)
    {
        rcl_arguments_t* global_args = null, local_args= GetNodeArguments(node.Handle);
        if (node.Options.UseGlobalArguments)
        {
            global_args = &node.Context.Handle.Object->global_arguments;
        }

        Utils.ResolveParameterOverrides(node.FullyQualifiedName, paramOverrides, local_args, global_args);

        node.CreateService<
            DescribeParametersService,
            DescribeParametersServiceRequest,
            DescribeParametersServiceResponse>($"{node.Name}/describe_parameters", this);
    }

    private unsafe static rcl_arguments_t* GetNodeArguments(SafeNodeHandle node)
    {
        var handle = rcl_node_get_options(node.Object);
        if (RosEnvironment.IsFoxy)
        {
            return &((RclFoxy.rcl_node_options_t*)handle)->arguments;
        }
        else if (RosEnvironment.IsHumble)
        {
            return &((RclHumble.rcl_node_options_t*)handle)->arguments;
        }

        RosEnvironment.ThrowUnsupportedDistribution();
        return null;
    }

    private static ParameterType GetParameterType<T>()
    {
        if (!_clrToRosMapping.TryGetValue(typeof(T), out var t))
        {
            throw new RclException($"Parameter type '{typeof(T).Name}' is not supported.");
        }

        return t;
    }

    private static void SetValue(ParameterValue value, object val)
    {
        var type = (ParameterType)value.Type;
        switch (type)
        {
            case ParameterType.Bool:
                value.BoolValue = (bool)val;
                break;
            case ParameterType.Integer:
                value.IntegerValue = (long)val;
                break;
            case ParameterType.Double:
                value.DoubleValue = (double)val;
                break;
            case ParameterType.String:
                value.StringValue = (string)val;
                break;
            case ParameterType.ByteArray:
                value.ByteArrayValue = (byte[])val;
                break;
            case ParameterType.BoolArray:
                value.BoolArrayValue = (bool[])val;
                break;
            case ParameterType.IntegerArray:
                value.IntegerArrayValue = (long[])val;
                break;
            case ParameterType.DoubleArray:
                value.DoubleArrayValue = (double[])val;
                break;
            case ParameterType.StringArray:
                value.StringArrayValue = (string[])val;
                break;
            default:
                throw new RclException($"Cannot set parameter value of type '{type}'.");
        };
    }

    /// <summary>
    /// 
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="name"></param>
    /// <param name="descriptor"></param>
    /// <returns></returns>
    public T DeclareParameter<T>(string name, T defaultValue, ParameterDescriptor? descriptor = null, bool ignoreOverride = false)
    {
        descriptor ??= new();

        var type = (byte)GetParameterType<T>();
        var pd = new ParameterDescriptor(
            name,
            type,
            descriptor.Description,
            descriptor.AdditionalConstraints,
            descriptor.ReadOnly,
            descriptor.FloatingPointRange,
            descriptor.IntegerRange);

        var p = new Parameter(pd, defaultValue!);
        if (!_parameters.TryAdd(name, p))
        {
            throw new RclException($"Parameter '{name}' is already declared.");
        }

        return (T)p.Value;
    }

    public DescribeParametersServiceResponse ProcessRequest(DescribeParametersServiceRequest request)
    {

        return new();
    }

    record Parameter(ParameterDescriptor Descriptor, object Value);
}
