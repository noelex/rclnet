using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime.Interop;

public enum MessageInitialization
{
    /// <summary>
    /// Initialize all fields of the message, either with the default value
    /// (if the field has one), or with an empty value (generally 0 or an
    /// empty string).
    /// </summary>
    All,

    /// <summary>
    /// Skip initialization of all fields of the message.  It is up to the user to
    /// ensure that all fields are initialized before use.
    /// </summary>
    Skip,

    /// <summary>
    /// Initialize all fields of the message to an empty value (generally 0 or an
    /// empty string).
    /// </summary>
    Zero,

    /// <summary>
    /// Initialize all fields of the message that have defaults; all other fields
    /// are left untouched.
    /// </summary>
    DefaultsOnly
}
