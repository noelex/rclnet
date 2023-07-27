namespace Rcl;

/// <summary>
/// The introspection state for a service client or service server.
/// </summary>
public enum ServiceIntrospectionState
{
    /// <summary>
    /// Introspection is disabled for the service.
    /// </summary>
    Disabled,

    /// <summary>
    /// The metadata is published for the service.
    /// </summary>
    MetadataOnly,

    /// <summary>
    /// The metadata and service request and response contents are published for the service.
    /// </summary>
    Full
}