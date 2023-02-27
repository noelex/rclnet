using Microsoft.Toolkit.HighPerformance;
using Microsoft.Toolkit.HighPerformance.Buffers;
using System.Text;

namespace Rosidl.Runtime.Interop
{
    /// <summary>
    /// Provides utility methods for marshaling strings.
    /// </summary>
    public static class StringMarshal
    {
        /// <summary>
        /// Creates a pooled string from the specified <see langword="string"/>.
        /// </summary>
        /// <param name="str">The <see langword="string"/> to be pooled.</param>
        /// <returns>A pooled string instance matching the content of the input string.</returns>
        public static string CreatePooledString(string str)
        {
            return StringPool.Shared.GetOrAdd(str);
        }

        /// <summary>
        /// Creates a pooled string from specified string buffer.
        /// </summary>
        /// <param name="str">A <see cref="ReadOnlySpan{Char}"/> containing the string to be pooled.</param>
        /// <returns>A pooled string instance matching the content of the input buffer.</returns>
        public static string CreatePooledString(ReadOnlySpan<char> str)
        {
            return StringPool.Shared.GetOrAdd(str);
        }

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer.
        /// </summary>
        /// <param name="buffer">A null terminated string buffer.</param>
        /// <param name="encoding">Encoding of the input buffer, defaults to <see cref="Encoding.UTF8"/>.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(byte* buffer, Encoding encoding)
        {
            if (buffer == null) return null;

            var len = 0;
            while (buffer[len] != 0) len++;

            var span = new Span<byte>(buffer, len);
            return StringPool.Shared.GetOrAdd(span, encoding ?? Encoding.UTF8);
        }

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer with UTF-8 encoding.
        /// </summary>
        /// <param name="buffer">A null terminated string buffer.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(byte* buffer)
            => CreatePooledString(buffer, Encoding.UTF8);

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer.
        /// </summary>
        /// <param name="buffer">The string buffer to be pooled.</param>
        /// <param name="length">Length of the string buffer in bytes.</param>
        /// <param name="encoding">Encoding of the input buffer.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(byte* buffer, int length, Encoding encoding)
        {
            if (buffer == null) return null;

            var span = new Span<byte>(buffer, length);
            return CreatePooledString(span, encoding);
        }

        /// <summary>
        /// Creates a pooled string from specified string buffer.
        /// </summary>
        /// <param name="buffer">The string buffer to be pooled.</param>
        /// <param name="encoding">Encoding of the input buffer.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer.
        /// </returns>
        public unsafe static string CreatePooledString(Span<byte> buffer, Encoding encoding)
        {
            return StringPool.Shared.GetOrAdd(buffer, encoding);
        }

        /// <summary>
        /// Creates a pooled string from specified string buffer.
        /// </summary>
        /// <param name="buffer">The string buffer to be pooled.</param>
        /// <param name="encoding">Encoding of the input buffer.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer.
        /// </returns>
        public unsafe static string CreatePooledString(Span<sbyte> buffer, Encoding encoding)
        {
            return CreatePooledString(buffer.AsBytes(), encoding);
        }

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer with UTF-8 encoding.
        /// </summary>
        /// <param name="buffer">The string buffer to be pooled.</param>
        /// <param name="length">Length of the string buffer in bytes.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(byte* buffer, int length)
            => CreatePooledString(buffer, length, Encoding.UTF8);



        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer.
        /// </summary>
        /// <param name="buffer">A null terminated string buffer.</param>
        /// <param name="encoding">Encoding of the input buffer, defaults to <see cref="Encoding.UTF8"/>.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(sbyte* buffer, Encoding encoding)
            => CreatePooledString((byte*)buffer, encoding);

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer with UTF-8 encoding.
        /// </summary>
        /// <param name="buffer">A null terminated string buffer.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(sbyte* buffer)
            => CreatePooledString((byte*)buffer);

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer.
        /// </summary>
        /// <param name="buffer">The string buffer to be pooled.</param>
        /// <param name="length">Length of the string buffer in bytes.</param>
        /// <param name="encoding">Encoding of the input buffer.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(sbyte* buffer, int length, Encoding encoding)
            => CreatePooledString((byte*)buffer, length, encoding);

        /// <summary>
        /// Creates a pooled string from specified unmanaged string buffer with UTF-8 encoding.
        /// </summary>
        /// <param name="buffer">The string buffer to be pooled.</param>
        /// <param name="length">Length of the string buffer in bytes.</param>
        /// <returns>
        /// A pooled string instance matching the content of the input buffer if <paramref name="buffer"/> is not <see langword="null"/>.
        /// Otherwise, <see langword="null"/>.
        /// </returns>
        public unsafe static string? CreatePooledString(sbyte* buffer, int length)
            => CreatePooledString((byte*)buffer, length);
    }
}
