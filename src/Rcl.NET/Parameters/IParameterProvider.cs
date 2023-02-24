namespace Rcl.Parameters;

public delegate ValidationResult OnParameterChangingDelegate(ReadOnlySpan<ParameterChangingInfo> parameters, object? state);

public interface IParameterProvider
{
    Variant Declare(ParameterDescriptor descriptor, Variant defaultValue, bool ignoreOverride = false);

    Variant Declare(string name, Variant defaultValue, bool ignoreOverride = false);

    Variant Declare(string name, ValueKind type, bool ignoreOverride = false);

    Variant Declare(ParameterDescriptor descriptor, bool ignoreOverride = false);

    Variant GetOrDeclare(string name, Variant defaultValue, bool ignoreOverride = false);

    Variant GetOrDeclare(string name, ValueKind type, bool ignoreOverride = false);

    Variant Get(string name);

    bool TryGet(string name, out Variant variant);

    void Set(string name, Variant value);

    void Set(IDictionary<string, Variant> parameters);

    void SetAtomically(IDictionary<string, Variant> parameters);

    Variant[] Get(params string[] names);

    IDictionary<string, Variant> GetByPrefix(string prefix);

    ParameterDescriptor Describe(string name);

    ParameterDescriptor[] Describe(params string[] names);

    void Undeclare(string name);

    bool IsDeclared(string name);

    IDisposable RegisterParameterChangingCallback(OnParameterChangingDelegate callback, object? state = null);
}
