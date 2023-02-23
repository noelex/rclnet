using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

public record NumberRange<T>
(
    T Min,
    T Max,
    T Step
) where T : INumber<T>
{
    public bool IsInRange(T v)
    {
        if (v == Min || v == Max)
        {
            return true;
        }

        if ((v < Min) || (v > Max))
        {
            return false;
        }

        if (Step == T.Zero)
        {
            return true;
        }

        if (((v - Min) % Step) == T.Zero)
        {
            return true;
        }

        return false;
    }
}
