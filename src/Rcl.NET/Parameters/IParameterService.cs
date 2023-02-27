namespace Rcl.Parameters;

/// <summary>
/// Delegate for handling parameter changing events.
/// </summary>
/// <param name="parameters">
/// A list of parameters being changed.
/// </param>
/// <param name="state">
/// A user provided state when registering the event.
/// </param>
/// <returns>
/// A <see cref="ValidationResult"/> for reporting the state of the validation.
/// Returning a <see cref="ValidationResult"/> with <see cref="ValidationResult.IsSuccessful"/>
/// set to <see langword="false"/> will prevent to paramter to be changed,
/// and the event will stop being propagated to other pending event handlers.
/// </returns>
public delegate ValidationResult ParameterChangingEventHandler(ReadOnlySpan<ParameterChangingInfo> parameters, object? state);

/// <summary>
/// Represents a service for accessing parameters of an <see cref="IRclNode"/>.
/// </summary>
public interface IParameterService
{
    /// <summary>
    /// Declare a parameter with specified <see cref="ParameterDescriptor"/>.
    /// </summary>
    /// <param name="descriptor">A <see cref="ParameterDescriptor"/> containing the specification of the parameter to be decalred.</param>
    /// <param name="defaultValue">The default value of the parameter.</param>
    /// <param name="ignoreOverride">
    /// If <see langword="false"/>, value provided by YAML configuration files or command-line arguments
    /// will override the default value provided here. Otherwise, those values are ignored.
    /// </param>
    /// <returns>The value of the declared parameter.</returns>
    Variant Declare(ParameterDescriptor descriptor, Variant defaultValue, bool ignoreOverride = false);

    /// <summary>
    /// Declare a parameter with specified name.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="defaultValue">The default value of the parameter.</param>
    /// <param name="ignoreOverride">
    /// If <see langword="false"/>, value provided by YAML configuration files or command-line arguments
    /// will override the default value provided here. Otherwise, those values are ignored.
    /// </param>
    /// <returns>The value of the declared parameter.</returns>
    Variant Declare(string name, Variant defaultValue, bool ignoreOverride = false);

    /// <summary>
    /// Declare a parameter with specified name.
    /// </summary>
    /// <remarks>
    /// The declared parameter will be given a default value of the specific <paramref name="type"/>,
    /// if no override value is provided.
    /// </remarks>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="type">The type of the parameter.</param>
    /// <param name="ignoreOverride">
    /// If <see langword="false"/>, value provided by YAML configuration files or command-line arguments
    /// will override the default value provided here. Otherwise, those values are ignored.
    /// </param>
    /// <returns>The value of the declared parameter.</returns>
    Variant Declare(string name, ValueKind type, bool ignoreOverride = false);

    /// <summary>
    /// Declare a parameter with specified <see cref="ParameterDescriptor"/>.
    /// </summary>
    /// <remarks>
    /// The declared parameter will be given a default value of <see cref="ParameterDescriptor.Type"/>,
    /// if no override value is provided.
    /// </remarks>
    /// <param name="descriptor">A <see cref="ParameterDescriptor"/> containing the specification of the parameter to be decalred.</param>
    /// <param name="ignoreOverride">
    /// If <see langword="false"/>, value provided by YAML configuration files or command-line arguments
    /// will override the default value provided here. Otherwise, those values are ignored.
    /// </param>
    /// <returns>The value of the declared parameter.</returns>
    Variant Declare(ParameterDescriptor descriptor, bool ignoreOverride = false);

    /// <summary>
    /// Tries to get the value of the specified parameter, and if not declared, declare the parameter with the <paramref name="defaultValue"/>.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="defaultValue">The default value of the parameter for declaring the parameter.</param>
    /// <param name="ignoreOverride">
    /// If <see langword="false"/>, value provided by YAML configuration files or command-line arguments
    /// will override the default value provided here. Otherwise, those values are ignored.
    /// </param>
    /// <returns>The value of the existing or newwly declared parameter.</returns>
    Variant GetOrDeclare(string name, Variant defaultValue, bool ignoreOverride = false);

    /// <summary>
    /// Tries to get the value of the specified parameter, and if not declared, declare the parameter with its value set to the default of <paramref name="type"/>.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="type">The type of the parameter.</param>
    /// <param name="ignoreOverride">
    /// If <see langword="false"/>, value provided by YAML configuration files or command-line arguments
    /// will override the default value provided here. Otherwise, those values are ignored.
    /// </param>
    /// <returns>The value of the existing or newwly declared parameter.</returns>
    Variant GetOrDeclare(string name, ValueKind type, bool ignoreOverride = false);

    /// <summary>
    /// Gets the value of the specified parameter.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <returns>The value of the parameter.</returns>
    Variant Get(string name);

    /// <summary>
    /// Tries to get the value of the specified parameter.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="variant">The value of the parameter.</param>
    /// <returns>
    /// <see langword="true"/> if the parameter exists, otherwise <see langword="false"/>
    /// </returns>
    bool TryGet(string name, out Variant variant);

    /// <summary>
    /// Sets the value of a declared parameter.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="value">The value of the parameter.</param>
    void Set(string name, Variant value);

    /// <summary>
    /// Sets the value of multiple parameters.
    /// </summary>
    /// <param name="parameters">A dictionary containing the parameters to be set.</param>
    void Set(IDictionary<string, Variant> parameters);

    /// <summary>
    /// Sets the value of multiple parameters atomically, that is, should any the parameter
    /// in the provided dictionary fails to be set, all parameters will reman intact.
    /// </summary>
    /// <param name="parameters">A dictionary containing the parameters to be set.</param>
    void SetAtomically(IDictionary<string, Variant> parameters);

    /// <summary>
    /// Gets the value of specified parameters.
    /// </summary>
    /// <param name="names">A list of parameter name.</param>
    /// <returns>An array containing the value of corresponding parameters.</returns>
    Variant[] Get(params string[] names);

    /// <summary>
    /// Gets the value of the parameters with specified prefix.
    /// </summary>
    /// <param name="prefix">The prefix of the parameter.</param>
    /// <returns>A dictionary of parameter names and values.</returns>
    IDictionary<string, Variant> GetByPrefix(string prefix);

    /// <summary>
    /// Gets the descriptor of a parameter.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <returns>The descriptor of the parameter.</returns>
    ParameterDescriptor Describe(string name);

    /// <summary>
    /// Gets the descriptors of specified parameters.
    /// </summary>
    /// <param name="names">A list of parameter name.</param>
    /// <returns>An array containing the descriptor of corresponding parameters.</returns>
    ParameterDescriptor[] Describe(params string[] names);

    /// <summary>
    /// Undeclare a parameter.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    void Undeclare(string name);

    /// <summary>
    /// Checks whether the specified parameter is declared.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <returns>
    /// <see langword="true"/> if the parameter is declared, otherwise <see langword="false"/>.
    /// </returns>
    bool IsDeclared(string name);

    /// <summary>
    /// Register a handler which will be called when any parameter is going to be changed.
    /// </summary>
    /// <param name="callback">
    /// The callback to be called when one or more parameters are going to be changed.
    /// </param>
    /// <param name="state">A custom state object to be passed to the callback.</param>
    /// <returns>An <see cref="IDisposable"/> which will unregister the event handler when disposed.</returns>
    IDisposable RegisterParameterChangingEvent(ParameterChangingEventHandler callback, object? state = null);
}
