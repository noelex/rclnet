namespace Rcl.Introspection;

internal unsafe static class TypeSupportIdentifier
{
    public static ReadOnlySpan<byte> Introspection => "rosidl_typesupport_introspection_c\0"u8;

    public static ReadOnlySpan<byte> FastRtps => "rosidl_typesupport_fastrtps_c\0"u8;
}
