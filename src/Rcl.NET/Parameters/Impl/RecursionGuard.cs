using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters.Impl;

ref struct RecursionGuard
{
    private ref bool _updating;

    public RecursionGuard(ref bool isUpdating)
    {
        _updating = ref isUpdating;

        if (_updating)
        {
            throw new RclException("Cannot set or declare parameters from within parameter changing callbacks.");
        }

        _updating = true;
    }

    public void Dispose()
    {
        _updating = false;
    }
}
