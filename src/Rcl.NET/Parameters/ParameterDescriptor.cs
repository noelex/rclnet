using Rcl.Runtime;

namespace Rcl.Parameters;

/// <summary>
/// Represents an ROS 2 parameter.
/// </summary>
public record ParameterDescriptor
{
    /// <summary>
    /// Create a new <see cref="ParameterDescriptor"/>.
    /// </summary>
    /// <param name="name">The name of the parameter.</param>
    /// <param name="type">The type of the parameter value.</param>
    /// <param name="readOnly">If <see langword="true"/> then the value cannot change after it has been initialized.</param>
    /// <param name="description">Description of the parameter, visible from introspection tools.</param>
    /// <param name="additionalConstraints">
    /// Plain English description of additional constraints which cannot be expressed
    /// with the available constraints, e.g. "only prime numbers".
    /// <para>
    /// By convention, this should only be used to clarify constraints which cannot
    /// be completely expressed with the other predefined parameter constraints.
    /// </para>
    /// </param>
    /// <param name="floatingPointRange">
    /// Restrict the range and step of the parameter value.
    /// <para>
    /// This is only effective when <paramref name="type"/> is <see cref="ValueKind.Double"/>.
    /// </para>
    /// </param>
    /// <param name="integerRange">
    /// Restrict the range and step of the parameter value.
    /// <para>
    /// This is only effective when <paramref name="type"/> is <see cref="ValueKind.Integer"/>.
    /// </para>
    /// </param>
    /// <param name="dynamicTyping">
    /// If <see langword="true"/>, the parameter is allowed to change type.
    /// </param>
    /// <exception cref="ArgumentException"></exception>
    public ParameterDescriptor(
        string name,
        ValueKind type,
        bool readOnly = false,
        [SupportedSinceDistribution(RosEnvironment.Humble)]
        bool dynamicTyping = false,
        string description = "",
        string additionalConstraints = "",
        NumberRange<double>? floatingPointRange = null,
        NumberRange<long>? integerRange = null)
    {
        if (dynamicTyping)
        {
            RosEnvironment.Require(RosEnvironment.Humble);

            if (type != ValueKind.Unknown)
            {
                throw new ArgumentException($"Parameter type must be '{ValueKind.Unknown}' if dynamic typing enabled.");
            }
        }

        if (floatingPointRange != null && type != ValueKind.Double)
        {
            throw new ArgumentException(
                $"{nameof(floatingPointRange)} can only be set when parameter type is '{ValueKind.Double}'.");
        }

        if (integerRange != null && type != ValueKind.Integer)
        {
            throw new ArgumentException(
                $"{nameof(integerRange)} can only be set when parameter type is '{ValueKind.Integer}'.");
        }

        if (!dynamicTyping)
        {
            if (type == ValueKind.Unknown)
            {
                throw new ArgumentException("Parameter type must be set if dynamic typing is disabled.");
            }
        }

        Name = name;
        Type = type;
        ReadOnly = readOnly;
        Description = description;
        AdditionalConstraints = additionalConstraints;
        FloatingPointRange = floatingPointRange;
        IntegerRange = integerRange;
        DynamicTyping = dynamicTyping;
    }

    /// <summary>
    /// The name of the parameter.
    /// </summary>
    public string Name { get; }

    /// <summary>
    /// The type of the parameter value.
    /// </summary>
    public ValueKind Type { get; }

    /// <summary>
    /// If <see langword="true"/> then the value cannot change after it has been initialized.
    /// </summary>
    public bool ReadOnly { get; }

    /// <summary>
    /// Description of the parameter, visible from introspection tools.
    /// </summary>
    public string Description { get; }

    /// <summary>
    /// Plain English description of additional constraints which cannot be expressed
    /// with the available constraints, e.g. "only prime numbers".
    /// </summary>
    /// <remarks>
    /// By convention, this should only be used to clarify constraints which cannot
    /// be completely expressed with the other predefined parameter constraints.
    /// </remarks>
    public string AdditionalConstraints { get; }

    /// <summary>
    /// Restrict the range and step of the parameter value.
    /// </summary>
    /// <remarks>
    /// This is only effective when <see cref="Type"/> is <see cref="ValueKind.Double"/>.
    /// </remarks>
    public NumberRange<double>? FloatingPointRange { get; }

    /// <summary>
    /// Restrict the range and step of the parameter value.
    /// </summary>
    /// <remarks>
    /// This is only effective when <see cref="Type"/> is <see cref="ValueKind.Integer"/>.
    /// </remarks>
    public NumberRange<long>? IntegerRange { get; }

    /// <summary>
    /// If <see langword="true"/>, the parameter is allowed to change type.
    /// </summary>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    public bool DynamicTyping { get; }
}