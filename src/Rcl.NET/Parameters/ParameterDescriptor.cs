using Rcl.Runtime;

namespace Rcl.Parameters;

/// <summary>
/// Represents an ROS 2 parameter.
/// </summary>
/// <param name="Name">The name of the parameter.</param>
/// <param name="Type">The type of the parameter value.</param>
/// <param name="ReadOnly">If <see langword="true"/> then the value cannot change after it has been initialized.</param>
/// <param name="Description">Description of the parameter, visible from introspection tools.</param>
/// <param name="AdditionalConstraints">
/// Plain English description of additional constraints which cannot be expressed
/// with the available constraints, e.g. "only prime numbers".
/// <para>
/// By convention, this should only be used to clarify constraints which cannot
/// be completely expressed with the other predefined parameter constraints.
/// </para>
/// </param>
/// <param name="FloatingPointRange">
/// Restrict the range and step of the parameter value.
/// <para>
/// This is only effective when <paramref name="Type"/> is <see cref="ValueKind.Double"/>.
/// </para>
/// </param>
/// <param name="IntegerRange">
/// Restrict the range and step of the parameter value.
/// <para>
/// This is only effective when <paramref name="Type"/> is <see cref="ValueKind.Integer"/>.
/// </para>
/// </param>
/// <param name="DynamicTyping">
/// If <see langword="true"/>, the parameter is allowed to change type.
/// </param>
public record ParameterDescriptor
(
    string Name,

    ValueKind Type,

    bool ReadOnly = false,

    string Description = "",

    string AdditionalConstraints = "",

    NumberRange<double>? FloatingPointRange = null,

    NumberRange<long>? IntegerRange = null,

    [SupportedSinceDistribution(RosEnvironment.Humble)]
    bool DynamicTyping = false
);