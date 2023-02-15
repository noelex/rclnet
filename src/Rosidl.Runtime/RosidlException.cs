using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime
{
    public class RosidlException : Exception
    {
        public RosidlException(string message) : base(message) { }
    }
}
