using Microsoft.Toolkit.HighPerformance.Buffers;
using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Messages.Rcl;
using System.Collections.Concurrent;
using System.Xml.Linq;

namespace Rcl.Parameters.Impl;

partial class ParameterService : IParameterService, IDisposable
{
    [ThreadStatic]
    private static bool _recursionFlag;

    private SpinLock _lock;

    private readonly RclNodeImpl _node;
    private readonly IDictionary<string, Variant> _overrides;
    private readonly ConcurrentDictionary<string, ParameterStore> _parameters = new();

    private readonly List<ParameterChangingCallback> _onParameterChangingCallbacks = new();

    internal unsafe ParameterService(RclNodeImpl node, IDictionary<string, Variant> paramOverrides)
    {
        _node = node;

        rcl_arguments_t* global_args = null, local_args = GetNodeArguments(node.Handle);
        if (node.Options.UseGlobalArguments)
        {
            global_args = &node.Context.Handle.Object->global_arguments;
        }

        _overrides = Utils.ResolveParameterOverrides(node.FullyQualifiedName, paramOverrides, local_args, global_args);

        var completelyInitialized = false;
        try
        {
            if (RosEnvironment.IsFoxy)
            {
                _describeParametersService = node.CreateService<
                    DescribeParametersServiceFoxy,
                    DescribeParametersServiceRequestFoxy,
                    DescribeParametersServiceResponseFoxy>($"{node.Name}/describe_parameters", this, new(qos: QosProfile.Parameters));
            }
            else
            {
                _describeParametersService = node.CreateService<
                    DescribeParametersService,
                    DescribeParametersServiceRequest,
                    DescribeParametersServiceResponse>($"{node.Name}/describe_parameters", this, new(qos: QosProfile.Parameters));
            }

            _listParametersService = node.CreateService<
                    ListParametersService,
                    ListParametersServiceRequest,
                    ListParametersServiceResponse>($"{node.Name}/list_parameters", this, new(qos: QosProfile.Parameters));
            _getParametersService = node.CreateService<
                    GetParametersService,
                    GetParametersServiceRequest,
                    GetParametersServiceResponse>($"{node.Name}/get_parameters", this, new(qos: QosProfile.Parameters));
            _getParameterTypesService = node.CreateService<
                    GetParameterTypesService,
                    GetParameterTypesServiceRequest,
                    GetParameterTypesServiceResponse>($"{node.Name}/get_parameter_types", this, new(qos: QosProfile.Parameters));
            _setParametersService = node.CreateService<
                    SetParametersService,
                    SetParametersServiceRequest,
                    SetParametersServiceResponse>($"{node.Name}/set_parameters", this, new(qos: QosProfile.Parameters));
            _setParametersAtomicallyService = node.CreateService<
                    SetParametersAtomicallyService,
                    SetParametersAtomicallyServiceRequest,
                    SetParametersAtomicallyServiceResponse>($"{node.Name}/set_parameters_atomically", this, new(qos: QosProfile.Parameters));

            _parameterEvents = node.CreatePublisher<ParameterEvent>("/parameter_events", new(qos: QosProfile.ParameterEvents));
            completelyInitialized = true;
        }
        finally
        {
            if (!completelyInitialized)
            {
                _describeParametersService?.Dispose();
                _listParametersService?.Dispose();
                _getParametersService?.Dispose();
                _getParameterTypesService?.Dispose();
                _setParametersService?.Dispose();
                _setParametersAtomicallyService?.Dispose();
                _parameterEvents?.Dispose();
            }
        }

    }

    private unsafe static rcl_arguments_t* GetNodeArguments(SafeNodeHandle node)
    {
        var handle = rcl_node_get_options(node.Object);
        if (RosEnvironment.IsFoxy)
        {
            return &((RclFoxy.rcl_node_options_t*)handle)->arguments;
        }
        else if (RosEnvironment.IsHumble || RosEnvironment.IsIron)
        {
            return &((RclHumble.rcl_node_options_t*)handle)->arguments;
        }

        throw new NotImplementedException();
    }

    private static Variant GetDefaultValue(ValueKind type)
    {
        return type switch
        {
            ValueKind.Bool => false,
            ValueKind.BoolArray => Array.Empty<bool>(),
            ValueKind.ByteArray => Array.Empty<byte>(),
            ValueKind.Double => 0d,
            ValueKind.DoubleArray => Array.Empty<double>(),
            ValueKind.Integer => 0L,
            ValueKind.IntegerArray => Array.Empty<long>(),
            ValueKind.String => string.Empty,
            ValueKind.StringArray => Array.Empty<string>(),
            ValueKind.Unknown => new(),
            _ => throw new RclException($"Value type '{type}' is not supported.")
        };
    }

    public IDisposable RegisterParameterChangingEvent(ParameterChangingEventHandler callback, object? state = null)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            var cb = new ParameterChangingCallback(this, callback, state);
            _onParameterChangingCallbacks.Add(cb);
            return cb;
        }
    }

    private void UnregisterParameterChangingCallback(ParameterChangingCallback cb)
    {
        using (ScopedLock.Lock(ref _lock))
        {
            _onParameterChangingCallbacks.Remove(cb);
        }
    }

    private ValidationResult NotifyParameterChanging(ReadOnlySpan<ParameterChangingInfo> parameters)
    {
        SpanOwner<ParameterChangingCallback> callbacksSnapshot;
        using (ScopedLock.Lock(ref _lock))
        {
            callbacksSnapshot = SpanOwner<ParameterChangingCallback>.Allocate(_onParameterChangingCallbacks.Count);
            for (var i = 0; i < _onParameterChangingCallbacks.Count; i++)
            {
                callbacksSnapshot.Span[i] = _onParameterChangingCallbacks[i];
            }
        }

        // Prevent callbacks from declaring / setting parameters.
        using var gurad = new RecursionGuard(ref _recursionFlag);
        var result = ValidationResult.Success();
        foreach (var cb in callbacksSnapshot.Span)
        {
            result = cb.Callback(parameters, cb.State);
            if (!result.IsSuccessful)
            {
                break;
            }
        }

        return result;
    }

    private ValidationResult DeclareCore(ParameterDescriptor descriptor, Variant defaultValue, bool ignoreOverride, out Variant value)
    {
        value = !ignoreOverride && _overrides.TryGetValue(descriptor.Name, out var v) ? v : defaultValue;
        var ps = new ParameterStore(descriptor);

        var result = ps.Initialize(value);
        if (!result.IsSuccessful)
        {
            return result;
        }

        using var info = SpanOwner<ParameterChangingInfo>.Allocate(1);
        info.Span[0] = new(ps.Descriptor, default, value);
        result = NotifyParameterChanging(info.Span);
        if (!result.IsSuccessful)
        {
            return result;
        }

        if (!_parameters.TryAdd(descriptor.Name, ps))
        {
            result = ValidationResult.Failure($"Parameter '{descriptor.Name}' is already declared.");
        }

        PublishNewParameter(ps);
        return result;
    }

    public Variant Declare(ParameterDescriptor descriptor, Variant defaultValue, bool ignoreOverride = false)
    {
        var result = DeclareCore(descriptor, defaultValue, ignoreOverride, out var value);
        EnsureSuccessful(result);

        return value;
    }

    public Variant Declare(string name, Variant defaultValue, bool ignoreOverride = false)
        => Declare(new ParameterDescriptor(name, defaultValue.Kind), defaultValue, ignoreOverride);

    public Variant Declare(string name, ValueKind type, bool ignoreOverride = false)
        => Declare(new(name, type), ignoreOverride);

    public Variant Declare(ParameterDescriptor descriptor, bool ignoreOverride = false)
        => Declare(descriptor, GetDefaultValue(descriptor.Type), ignoreOverride);

    public void Undeclare(string name)
    {
        // Just checking whether the parameter is declared.
        var ps = GetStore(name);
        _parameters.Remove(name, out _);
        PublishRemovedParameter(ps);
    }

    public bool IsDeclared(string name)
    {
        return _parameters.ContainsKey(name);
    }

    public Variant Get(string name)
        => GetStore(name).Value;

    public bool TryGet(string name, out Variant value)
    {
        if (_parameters.TryGetValue(name, out var ps))
        {
            value = ps.Value;
            return true;
        }

        value = default;
        return false;
    }

    public void Set(string name, Variant value)
    {
        var result = SetAndNotify(name, value);
        EnsureSuccessful(result);
    }

    public Variant GetOrDeclare(string name, Variant defaultValue, bool ignoreOverride = false)
    {
        if (TryGet(name, out var val))
        {
            return val;
        }

        return Declare(name, defaultValue, ignoreOverride);
    }

    public Variant GetOrDeclare(string name, ValueKind type, bool ignoreOverride = false)
    {
        if (TryGet(name, out var val))
        {
            return val;
        }

        return Declare(name, type, ignoreOverride);
    }

    public void Set(IDictionary<string, Variant> parameters)
    {
        foreach (var (k, v) in parameters)
        {
            Set(k, v);
        }
    }

    public void SetAtomically(IDictionary<string, Variant> parameters)
    {
        EnsureSuccessful(SetAndNotify(parameters));
    }

    private ValidationResult SetAndNotify(IDictionary<string, Variant> parameters)
    {
        ValidationResult result;
        var temp = new Dictionary<string, ParameterStore>();
        using var info = SpanOwner<ParameterChangingInfo>.Allocate(parameters.Count);

        var i = 0;

        foreach (var (k, v) in parameters)
        {
            result = TryGetStore(k, out var ps);
            if (!result.IsSuccessful)
            {
                return result;
            }

            result = ps.Validate(v);
            if (!result.IsSuccessful)
            {
                return result;
            }

            info.Span[i++] = new(ps.Descriptor, ps.Value, v);
            temp[k] = ps;
        }

        result = NotifyParameterChanging(info.Span);
        if (!result.IsSuccessful)
        {
            return result;
        }

        foreach (var (k, v) in parameters)
        {
            temp[k].UnsafeSet(v);
        }

        PublishChangedParameters(temp.Values);
        return result;
    }

    private ValidationResult SetAndNotify(string name, Variant value)
    {
        var result = TryGetStore(name, out var ps);
        if (!result.IsSuccessful)
        {
            return result;
        }

        result = ps.Validate(value);
        if (!result.IsSuccessful)
        {
            return result;
        }

        using var info = SpanOwner<ParameterChangingInfo>.Allocate(1);
        info.Span[0] = new(ps.Descriptor, ps.Value, value);
        result = NotifyParameterChanging(info.Span);
        if (!result.IsSuccessful)
        {
            return result;
        }

        ps.UnsafeSet(value);

        PublishChangedParameters(Enumerable.Repeat(ps, 1));
        return result;
    }

    public Variant[] Get(params string[] names)
    {
        var result = new Variant[names.Length];
        for (var i = 0; i < result.Length; i++)
        {
            result[i] = Get(names[i]);
        }

        return result;
    }

    public IDictionary<string, Variant> GetByPrefix(string prefix)
    {
        prefix = prefix == "" ? prefix : prefix + ".";
        var result = new Dictionary<string, Variant>();
        foreach (var (k, v) in _parameters)
        {
            if (k.StartsWith(prefix))
            {
                result[k.Substring(prefix.Length)] = v.Value;
            }
        }

        return result;
    }

    public ParameterDescriptor Describe(string name)
    {
        return GetStore(name).Descriptor;
    }

    public ParameterDescriptor[] Describe(params string[] names)
    {
        var result = new ParameterDescriptor[names.Length];
        for (var i = 0; i < result.Length; i++)
        {
            result[i] = Describe(names[i]);
        }

        return result;
    }

    private ParameterStore GetStore(string name)
    {
        var result = TryGetStore(name, out var p);
        EnsureSuccessful(result);
        return p;
    }

    private ValidationResult TryGetStore(string name, out ParameterStore ps)
    {
        if (!_parameters.TryGetValue(name, out ps!))
        {
            return ValidationResult.Failure($"Parameter '{name}' is not declared.");
        }

        return ValidationResult.Success();
    }

    private static void EnsureSuccessful(ValidationResult result)
    {
        if (!result.IsSuccessful)
        {
            throw new RclException(result.Message);
        }
    }

    public void Dispose()
    {
        ShutdownComms();
    }

    private record ParameterChangingCallback(ParameterService Provider, ParameterChangingEventHandler Callback, object? State) : IDisposable
    {
        public void Dispose()
        {
            Provider.UnregisterParameterChangingCallback(this);
        }
    }
}
