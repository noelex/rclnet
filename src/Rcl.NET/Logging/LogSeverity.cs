using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Logging;

public enum LogSeverity
{
    Unknown = 0,
    Debug = 10,
    Information = 20,
    Warning = 30,
    Error = 40,
    Fatal = 50
}
