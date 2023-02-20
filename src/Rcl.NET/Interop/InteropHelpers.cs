using Microsoft.Toolkit.HighPerformance.Buffers;
using System.Text;

namespace Rcl.Interop;

static class InteropHelpers
{
    public static rmw_time_t ToRmwTime(this TimeSpan t)
    {
        rmw_time_t rt;
        rt.sec = (ulong)t.TotalSeconds;
        rt.nsec = (ulong)((t.TotalSeconds - rt.sec) * 1000 * 1000 * 1000);
        return rt;
    }

    public static TimeSpan ToTimeSpan(this rmw_time_t t)
    {
        return TimeSpan.FromSeconds(t.sec + ((double)t.nsec / 1000 / 1000 / 1000));
    }

    public static TimeSpan ToTimeSpan(this rcl_time_point_value_t t)
    {
        return TimeSpan.FromMicroseconds((double)t.Value / 1000);
    }

    public static int GetUtf8BufferSize(string str, bool zeroTerminated = true)
    {
        return Encoding.UTF8.GetByteCount(str) + (zeroTerminated ? 1 : 0);
    }

    public static int GetUtf8BufferSize(string[] strArray, bool zeroTerminated = true)
    {
        var sum = 0;
        foreach (var str in strArray)
        {
            sum += GetUtf8BufferSize(str, zeroTerminated);
        }
        return sum;
    }

    public static void FillUtf8Buffer(string src, Span<byte> dest)
    {
        Encoding.UTF8.GetBytes(src, dest);
    }

    public static void FillUtf8Buffer(string[] src, Span<byte> dest, bool zeroTerminated = true)
    {
        var offset = 0;
        var terminator = zeroTerminated ? 1 : 0;
        foreach (var str in src)
        {
            offset += Encoding.UTF8.GetBytes(str, dest[offset..]) + terminator;
        }
    }

    public static void FillUtf8Buffer(string[] src, Span<byte> dest, Span<int> offsets, bool zeroTerminated = true)
    {
        var offset = 0;
        var terminator = zeroTerminated ? 1 : 0;
        for (var i = 0; i < src.Length; i++)
        {
            offsets[i] = offset;
            offset += Encoding.UTF8.GetBytes(src[i], dest[offset..]) + terminator;
        }
    }

    /// <summary>
    /// Encode string array with UTF8 and write output into <paramref name="dest"/>, then write pointers to each string in the <paramref name="dest"/> buffer into <paramref name="pointers"/>.
    /// </summary>
    /// <remarks>
    /// This method expects <paramref name="dest"/> to be allocated on stack, otherwise pointers in <paramref name="pointers"/> may become inaccessible after this method returns.
    /// </remarks>
    /// <param name="src"></param>
    /// <param name="dest"></param>
    /// <param name="pointers"></param>
    /// <param name="zeroTerminated"></param>
    public static unsafe void FillUtf8Buffer(string[] src, Span<byte> dest, byte** pointers, bool zeroTerminated = true)
    {
        var offset = 0;
        var terminator = zeroTerminated ? 1 : 0;

        fixed (byte* pDest = dest)
        {
            for (var i = 0; i < src.Length; i++)
            {
                pointers[i] = pDest + offset;
                offset += Encoding.UTF8.GetBytes(src[i], dest[offset..]) + terminator;
            }
        }

    }
}