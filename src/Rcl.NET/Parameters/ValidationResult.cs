namespace Rcl.Parameters;

/// <summary>
/// Represents the result of parameter validation.
/// </summary>
/// <param name="IsSuccessful">Indicates whether the validation is successful.</param>
/// <param name="Message">
/// Reason why the setting was either successful or a failure. This should only be
/// used for logging and user interfaces.
/// </param>
public record struct ValidationResult(bool IsSuccessful, string Message = "")
{
    /// <summary>
    /// Create a new <see cref="ValidationResult"/> with <see cref="IsSuccessful"/> set to <see langword="true"/>.
    /// </summary>
    /// <param name="message">
    /// Reason why the setting was successful. This should only be
    /// used for logging and user interfaces.
    /// </param>
    /// <returns>A successful <see cref="ValidationResult"/>.</returns>
    public static ValidationResult Success(string message = "") => new(true, message);

    /// <summary>
    /// Create a new <see cref="ValidationResult"/> with <see cref="IsSuccessful"/> set to <see langword="false"/>.
    /// </summary>
    /// <param name="message">
    /// Reason why the setting was failed. This should only be
    /// used for logging and user interfaces.
    /// </param>
    /// <returns>A failure <see cref="ValidationResult"/>.</returns>
    public static ValidationResult Failure(string message = "") => new(false, message);
}