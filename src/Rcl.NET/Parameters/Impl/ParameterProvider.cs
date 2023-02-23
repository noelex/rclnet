using Rcl.Interop;
using Rcl.SafeHandles;

namespace Rcl.Parameters.Impl;

internal class ParameterProvider : IParameterProvider
{
    private SpinLock _lock;
    private readonly ParameterDictionary _overrides = new();
    private readonly Dictionary<string, Parameter> _parameters = new();

    internal unsafe ParameterProvider(RclNodeImpl node, ParameterDictionary paramOverrides)
    {
        rcl_arguments_t* global_args = null, local_args = GetNodeArguments(node.Handle);
        if (node.Options.UseGlobalArguments)
        {
            global_args = &node.Context.Handle.Object->global_arguments;
        }

        _overrides = Utils.ResolveParameterOverrides(node.FullyQualifiedName, paramOverrides, local_args, global_args);
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

    private static Variant GetDefaultValue(ValueType type)
    {
        return type switch
        {
            ValueType.Bool => false,
            ValueType.BoolArray => Array.Empty<bool>(),
            ValueType.ByteArray => Array.Empty<byte>(),
            ValueType.Double => 0d,
            ValueType.DoubleArray => Array.Empty<double>(),
            ValueType.Integer => 0L,
            ValueType.IntegerArray => Array.Empty<long>(),
            ValueType.String => string.Empty,
            ValueType.StringArray => Array.Empty<string>(),
            ValueType.Unknown => new(),
            _ => throw new RclException($"Value type '{type}' is not supported.")
        };
    }

    private void Validate(ParameterDescriptor descriptor, Variant value, bool verifyReadOnly)
    {
        if (verifyReadOnly && descriptor.ReadOnly)
        {
            throw new RclException($"Parameter is read-only.");
        }

        if (descriptor.Type != value.Type && !descriptor.DynamicTyping)
        {
            throw new RclException($"Parameter of type '{descriptor.Type}' " +
                $"cannot be assigned with value of type '{value.Type}'.");
        }

        if (value.Type == ValueType.Integer && descriptor.IntegerRange != null)
        {
            if (!descriptor.IntegerRange.IsInRange((long)value))
            {
                throw new RclException($"Parameter value is out of range.");
            }
        }

        if (value.Type == ValueType.Double && descriptor.FloatingPointRange != null)
        {
            if (!descriptor.FloatingPointRange.IsInRange((double)value))
            {
                throw new RclException($"Parameter value is out of range.");
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="name"></param>
    /// <param name="descriptor"></param>
    /// <returns></returns>
    public Variant Declare(ParameterDescriptor descriptor, Variant defaultValue, bool ignoreOverride = false)
    {
        if (!descriptor.DynamicTyping)
        {
            if (descriptor.Type == ValueType.Unknown)
            {
                throw new RclException($"Cannot declare parameter of specific type '{descriptor.Type}' when dynamic typing is enabled.");
            }
        }

        var value = !ignoreOverride && _overrides.TryGetValue(descriptor.Name, out var v) ? v : defaultValue;
        Validate(descriptor, value, false);

        using (ScopedLock.Lock(ref _lock))
        {
            var p = new Parameter(descriptor, value);
            if (!_parameters.TryAdd(descriptor.Name, p))
            {
                throw new RclException($"Parameter '{descriptor.Name}' is already declared.");
            }

            return value;
        }
    }

    public Variant Declare(string name, Variant defaultValue, bool ignoreOverride = false)
        => Declare(new ParameterDescriptor(name, defaultValue.Type), defaultValue, ignoreOverride);

    public Variant Declare(string name, ValueType type, bool ignoreOverride = false)
        => Declare(new(name, type), ignoreOverride);

    public Variant Declare(ParameterDescriptor descriptor, bool ignoreOverride = false)
        => Declare(descriptor, GetDefaultValue(descriptor.Type), ignoreOverride);

    public Parameter Get(string name)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            if (_parameters.TryGetValue(name, out var p))
            {
                if (p.Value.Type != ValueType.Unknown || p.Descriptor.DynamicTyping)
                {
                    return p;
                }
                else
                {
                    throw new RclException($"Parameter '{name}' is not initialized yet.");
                }
            }

            throw new RclException($"Parameter '{name}' is not declared yet.");
        }
    }

    public bool TryGet(string name, out Parameter parameter)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            parameter = Parameter.Empty;
            if (_parameters.TryGetValue(name, out var p))
            {
                if (p.Value.Type != ValueType.Unknown || p.Descriptor.DynamicTyping)
                {
                    parameter = p;
                    return true;
                }
                else
                {
                    return false;
                }
            }

            return false;
        }
    }

    public void Set(Parameter parameter)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            if (!_parameters.TryGetValue(parameter.Descriptor.Name, out var oldValue))
            {
                throw new RclException($"Parameter '{parameter.Descriptor.Name}' is not declared yet.");
            }

            if (oldValue.Descriptor != parameter.Descriptor)
            {
                throw new RclException("Trying to update parameter with a differnet descriptor.");
            }

            Validate(parameter.Descriptor, parameter.Value, true);
            _parameters[parameter.Descriptor.Name] = parameter;
        }
    }

    public void Undeclare(string name)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            if (!_parameters.TryGetValue(name, out var p))
            {
                throw new RclException($"Parameter '{name}' is not declared yet.");
            }

            if (p.Descriptor.ReadOnly)
            {
                throw new RclException($"Parameter '{name}' is read-only.");
            }

            if (!p.Descriptor.DynamicTyping)
            {
                throw new RclException($"Cannot undeclare statically typed parameter '{name}'.");
            }

            _parameters.Remove(name, out _);
        }
    }

    public bool IsDeclared(string name)
    {
        using(ScopedLock.Lock(ref _lock))
        {
            return _parameters.ContainsKey(name);
        }
    }

    public Variant GetValue(string name)
        => Get(name).Value;

    public bool TryGetValue(string name, out Variant variant)
    {
        if (TryGet(name, out var p))
        {
            variant = p.Value;
            return true;
        }

        variant = new();
        return false;
    }

    public void Set(string name, Variant value)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            if (!_parameters.TryGetValue(name, out var oldValue))
            {
                throw new RclException($"Parameter '{name}' is not declared yet.");
            }

            if (oldValue.Descriptor.Type != value.Type && !oldValue.Descriptor.DynamicTyping)
            {
                throw new RclException("Trying to update parameter with a differnet type.");
            }

            Validate(oldValue.Descriptor, value, true);
            _parameters[oldValue.Descriptor.Name] = new(oldValue.Descriptor, value);
        }
    }
}
