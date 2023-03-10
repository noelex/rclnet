using Microsoft.Toolkit.HighPerformance.Buffers;
using Rcl.Interop;
using System.Runtime.CompilerServices;
using System.Text;

namespace Rcl.Logging.Impl;

internal class RcutilsLoggerFactory : IRclLoggerFactory
{
    public IRclLogger CreateLogger(string name)
    {
        return new RcutilsLogger(name);
    }
}

internal class RcutilsLogger : IRclLogger
{
    private readonly string _name;

    public RcutilsLogger(string name)
    {
        _name = name;
    }

    public string Name => _name;

    public void Log(LogSeverity severity, string? message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        message ??= "";

        var bufferSize =
                InteropHelpers.GetUtf8BufferSize(_name) +
                InteropHelpers.GetUtf8BufferSize(functionName) +
                InteropHelpers.GetUtf8BufferSize(file) +
                InteropHelpers.GetUtf8BufferSize(message);

        using var bufferOwner = SpanOwner<byte>.Allocate(bufferSize, AllocationMode.Clear);
        var buffer = bufferOwner.Span;

        int fileOffset, functionNameOffset, catergoryOffset, messageOffset;
        var offset = 0;

        fileOffset = offset;
        offset += Encoding.UTF8.GetBytes(file, buffer[offset..]) + 1;

        functionNameOffset = offset;
        offset += Encoding.UTF8.GetBytes(functionName, buffer[offset..]) + 1;

        catergoryOffset = offset;
        offset += Encoding.UTF8.GetBytes(_name, buffer[offset..]) + 1;

        messageOffset = offset;
        offset += Encoding.UTF8.GetBytes(message, buffer[offset..]) + 1;

        unsafe
        {
            fixed (byte* pBuf = buffer)
            {
                rcutils_log_location_t location;
                location.line_number = (uint)lineNumber;
                location.function_name = pBuf + functionNameOffset;
                location.file_name = pBuf + fileOffset;

                rcutils_log(&location, (int)severity, pBuf + catergoryOffset, pBuf + messageOffset, nint.Zero);
            }
        }
    }
}
