namespace Rcl.Parameters;

/// <summary>
/// Encapsulates the information of a changing parameter.
/// </summary>
/// <param name="Descriptor">The descriptor the of changing parameter.</param>
/// <param name="OldValue">
/// The old value of the parameter.
/// If the parameter is being declared, <paramref name="OldValue"/> will be of type <see cref="ValueKind.Unknown"/>.
/// </param>
/// <param name="NewValue">
/// The new value of the parameter.
/// If the parameter is being undeclared, <paramref name="NewValue"/> will be of type <see cref="ValueKind.Unknown"/>.
/// </param>
public record struct ParameterChangingInfo(ParameterDescriptor Descriptor, Variant OldValue, Variant NewValue);
