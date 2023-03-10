using Microsoft.Toolkit.HighPerformance.Buffers;
using Rcl.Interop;
using System.Runtime.CompilerServices;
using System.Text;

namespace Rcl.Logging.Impl;

internal class RcutilsLoggerFactory : IRclLoggerFactory
{
    private readonly RclContext _syncContext;

    public RcutilsLoggerFactory(RclContext syncContext)
    {
        _syncContext = syncContext;
    }

    public IRclLogger CreateLogger(string name)
    {
        return new RcutilsLogger(name);
    }
}

class LogEntry
{
    public LogSeverity Severity { get; set; }

    public string Category { get; set; } = null!;

    public string Message { get; set; } = null!;

    public string FilePath { get; set; } = null!;

    public string MemberName { get; set; } = null!;

    public int LineNumber { get; set; }

    public void Reset()
    {
        Severity = LogSeverity.Unknown;
        Message = null!;
        FilePath = null!;
        MemberName = null!;
        LineNumber = 0;
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
        var entry = ObjectPool.Rent<LogEntry>();
        entry.Severity = severity;
        entry.Message = message ?? "";
        entry.FilePath = file;
        entry.LineNumber = lineNumber;
        entry.Category = _name;
        entry.MemberName = functionName;

        ThreadPool.UnsafeQueueUserWorkItem(Log, entry, true);
    }

    private static void Log(LogEntry entry)
    {
        try
        {
            var bufferSize =
                InteropHelpers.GetUtf8BufferSize(entry.Category) +
                InteropHelpers.GetUtf8BufferSize(entry.MemberName) +
                InteropHelpers.GetUtf8BufferSize(entry.FilePath) +
                InteropHelpers.GetUtf8BufferSize(entry.Message);

            using var bufferOwner = SpanOwner<byte>.Allocate(bufferSize, AllocationMode.Clear);
            var buffer = bufferOwner.Span;

            //Span<byte> buffer = stackalloc byte[bufferSize];

            int fileOffset, functionNameOffset, catergoryOffset, messageOffset;
            var offset = 0;

            fileOffset = offset;
            offset += Encoding.UTF8.GetBytes(entry.FilePath, buffer[offset..]) + 1;

            functionNameOffset = offset;
            offset += Encoding.UTF8.GetBytes(entry.MemberName, buffer[offset..]) + 1;

            catergoryOffset = offset;
            offset += Encoding.UTF8.GetBytes(entry.Category, buffer[offset..]) + 1;

            messageOffset = offset;
            offset += Encoding.UTF8.GetBytes(entry.Message, buffer[offset..]) + 1;

            unsafe
            {
                fixed (byte* pBuf = buffer)
                {
                    rcutils_log_location_t location;
                    location.line_number = (uint)entry.LineNumber;
                    location.function_name = pBuf + functionNameOffset;
                    location.file_name = pBuf + fileOffset;

                    rcutils_log(&location, (int)entry.Severity, pBuf + catergoryOffset, pBuf + messageOffset, nint.Zero);
                }
            }
        }
        finally
        {
            entry.Reset();
            ObjectPool.Return(entry);
        }
    }
}
