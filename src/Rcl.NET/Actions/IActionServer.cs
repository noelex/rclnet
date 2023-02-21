using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Actions;

public interface IActionServer : IRclObject
{
    string Name { get; }
}
