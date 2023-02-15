using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime.Interop;

public enum FieldType:byte
{
    Float = 1,
    Double = 2,
    LongDouble = 3,
    Char = 4,
    WChar = 5,
    Boolean = 6,
    Octet = 7,
    UInt8 = 8,
    Int8 = 9,
    UInt16 = 10,
    Int16 = 11,
    UInt32 = 12,
    Int32 = 13,
    UInt64 = 14,
    Int64 = 15,
    String = 16,
    WString = 17,
    Message = 18,

    [Obsolete("Use FieldType.Float instead.")]
    Float32 = 1,

    [Obsolete("Use FieldType.Double instead.")]
    Float64 = 2,

    [Obsolete("Use FieldType.Boolean instead.")]
    Bool = 6,

    [Obsolete("Use FieldType.UInt8 instead.")]
    Byte = 7
}
