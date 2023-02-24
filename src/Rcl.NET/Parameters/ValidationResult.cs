using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters;

public record struct ValidationResult(bool IsSuccessful, string Message = "")
{
    public static ValidationResult Success() => new(true);
    public static ValidationResult Failure(string message) => new(false, message);
}