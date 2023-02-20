using Rcl.Interop;
using Rosidl.Runtime.Interop;

namespace Rcl;

public class RclException : Exception
{
    public RclException(int errorCode)
        : base(TryGetErrorMessage(errorCode))
    {
        Data[nameof(ErrorCode)] = errorCode;
    }

    public RclException(string message)
        :base(message)
    {
        Data[nameof(ErrorCode)] = -1;
    }

    static unsafe string? TryGetErrorMessage(int errorCode)
    {
        if (rcutils_error_is_set())
        {
            var s = rcutils_get_error_string();
            rcutils_reset_error();
            return StringMarshal.CreatePooledString((byte*)s.str);
        }
        return $"RCL returned error code {errorCode}.";
    }

    internal static void ThrowIfNonSuccess(rcl_ret_t errorCode)
    {
        if (errorCode != 0)
            throw new RclException((int)errorCode);
    }

    public int ErrorCode => (int)Data[nameof(ErrorCode)]!;
}