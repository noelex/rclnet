using Rosidl.Runtime.Interop;
using System.Net;
using System.Net.Sockets;
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
        return Encoding.UTF8.GetMaxByteCount(str.Length) + (zeroTerminated ? 1 : 0);
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

    public static unsafe NetworkFlowEndpoint[] ConvertNetworkFlowEndpoints(ref RclHumble.rmw_network_flow_endpoint_array_t endpoints)
    {
        if (endpoints.size == 0)
        {
            return Array.Empty<NetworkFlowEndpoint>();
        }

        var results = new NetworkFlowEndpoint[endpoints.size];
        for (var i = 0; i < results.Length; i++)
        {
            ref var ep = ref endpoints.network_flow_endpoint[i];
            var addressFamily = ep.internet_protocol switch
            {
                RclHumble.rmw_internet_protocol_t.RMW_INTERNET_PROTOCOL_IPV4 => AddressFamily.InterNetwork,
                RclHumble.rmw_internet_protocol_t.RMW_INTERNET_PROTOCOL_IPV6 => AddressFamily.InterNetworkV6,
                RclHumble.rmw_internet_protocol_t.RMW_INTERNET_PROTOCOL_UNKNOWN => AddressFamily.Unknown,
                _ => throw new NotSupportedException($"Unsupported rmw_internet_protocol_t value '{ep.internet_protocol}'.")
            };

            var transportProtocol = ep.transport_protocol switch
            {
                RclHumble.rmw_transport_protocol_t.RMW_TRANSPORT_PROTOCOL_UDP => ProtocolType.Udp,
                RclHumble.rmw_transport_protocol_t.RMW_TRANSPORT_PROTOCOL_TCP => ProtocolType.Tcp,
                RclHumble.rmw_transport_protocol_t.RMW_TRANSPORT_PROTOCOL_UNKNOWN => ProtocolType.Unknown,
                _ => throw new NotSupportedException($"Unsupported rmw_transport_protocol_t value '{ep.internet_protocol}'.")
            };

            IPAddress address;
            fixed (byte* addr = ep.internet_address)
            {
                var str = CreatePooledString(addr, 48)!;
                if (string.IsNullOrEmpty(str))
                {
                    address = IPAddress.None;
                }
                else
                {
                    address = IPAddress.Parse(CreatePooledString(addr, 48)!);
                }
            }

            results[i] = new NetworkFlowEndpoint(transportProtocol,
                new IPEndPoint(address, ep.transport_port), ep.flow_label, ep.dscp);
        }

        return results;

        static string CreatePooledString(byte* buffer, int maxLength)
        {
            var i = 0;
            while (i < maxLength && buffer[i] != '\0') i++;

            return StringMarshal.CreatePooledString(buffer, i)!;
        }
    }
}