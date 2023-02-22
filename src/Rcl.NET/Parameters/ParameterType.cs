namespace Rcl.Parameters;

/// <summary>
/// These types correspond to the value that is set in the ParameterValue message.
/// </summary>
public enum ParameterType : byte
{
    /// <summary>
    /// Default value, which implies this is not a valid parameter.
    /// </summary>
    Unknown = Rosidl.Messages.Rcl.ParameterType.PARAMETER_NOT_SET,

    Bool = Rosidl.Messages.Rcl.ParameterType.PARAMETER_BOOL,

    Integer = Rosidl.Messages.Rcl.ParameterType.PARAMETER_INTEGER,
    Double = Rosidl.Messages.Rcl.ParameterType.PARAMETER_DOUBLE,
    String = Rosidl.Messages.Rcl.ParameterType.PARAMETER_STRING,
    ByteArray = Rosidl.Messages.Rcl.ParameterType.PARAMETER_BYTE_ARRAY,
    BoolArray = Rosidl.Messages.Rcl.ParameterType.PARAMETER_BOOL_ARRAY,
    IntegerArray = Rosidl.Messages.Rcl.ParameterType.PARAMETER_INTEGER_ARRAY,
    DoubleArray = Rosidl.Messages.Rcl.ParameterType.PARAMETER_DOUBLE_ARRAY,
    StringArray = Rosidl.Messages.Rcl.ParameterType.PARAMETER_STRING_ARRAY,
}
