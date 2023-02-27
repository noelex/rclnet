using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

/// <summary>
/// Represents a range of numeric values.
/// </summary>
/// <typeparam name="T">The type of the numeric value.</typeparam>
/// <param name="Min">Inclusive minimum value of the range.</param>
/// <param name="Max">Inclusive maximum value of the range.</param>
/// <param name="Step">Size of valid steps between the <paramref name="Min"/> and <paramref name="Max"/> bound.
/// <para>
/// Step is considered to be a magnitude, therefore negative values are treated
/// the same as positive values, and a step value of zero implies a continuous
/// range of values.
/// </para>
/// <para>
/// Ideally, the step would be less than or equal to the distance between the
/// bounds, as well as an even multiple of the distance between the bounds, but
/// neither are required.
/// </para>
/// <para>
/// If the absolute value of the step is larger than or equal to the distance
/// between the two bounds, then the bounds will be the only valid values. e.g. if
/// the range is defined as {from_value: 1.0, to_value: 2.0, step: 5.0} then the
/// valid values will be 1.0 and 2.0.
/// </para>
/// <para>
/// If the step is less than the distance between the bounds, but the distance is
/// not a multiple of the step, then the "to" bound will always be a valid value,
/// e.g. if the range is defined as {from_value: 2.0, to_value: 5.0, step: 2.0}
/// then the valid values will be 2.0, 4.0, and 5.0.
/// </para>
/// </param>
public record NumberRange<T>
(
    T Min,
    T Max,
    T Step
) where T : INumber<T>
{
    /// <summary>
    /// Checks whether the specified value is in the range defined by current <see cref="NumberRange{T}"/>.
    /// </summary>
    /// <param name="v">The value to be checked.</param>
    /// <returns>
    /// <see langword="true"/> if the value in the range, otherwise <see langword="false"/>.
    /// </returns>
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
