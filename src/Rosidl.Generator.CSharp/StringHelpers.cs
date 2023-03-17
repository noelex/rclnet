using System.Text;

namespace Rosidl.Generator.CSharp;
internal static class StringHelpers
{
    public static string ToPascalCase(this string snake)
        => snake.ConvertCase(true);

    public static string ToCamelCase(this string snake)
        => snake.ConvertCase(false);

    private static string ConvertCase(this string snake, bool upperFirstCharacter)
    {
        var sb = new StringBuilder();
        var upper = upperFirstCharacter;
        var letterEncountered = false;

        for (var i = 0; i < snake.Length; i++)
        {
            if (snake[i] is '_' && letterEncountered)
            {
                upper = true;
            }
            else if (char.IsLetterOrDigit(snake[i]))
            {
                letterEncountered = true;

                if (upper && !char.IsUpper(snake[i]))
                {
                    sb.Append(char.ToUpper(snake[i]));
                    upper = false;
                }
                else
                {
                    sb.Append(snake[i]);
                }
            }
        }

        return sb.ToString();
    }

    public static string Escape(this string raw)
        => raw.Replace("\\", @"\\")
              .Replace("\t", @"\t")
              .Replace("\n", @"\n")
              .Replace("\r", @"\r")
              .Replace("\f", @"\f")
              .Replace("\b", @"\b")
              .Replace("\"", @"\""");
}
