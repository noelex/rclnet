using Microsoft.CodeAnalysis.Text;
using Microsoft.CodeAnalysis;
using System;
using System.Collections.Generic;
using System.Text;
using System.Text.RegularExpressions;

namespace Rosidl.Runtime.SourceGeneration;

[Generator]
public class CstSourceGenerator : ISourceGenerator
{
    private static readonly Regex StatementPattern =
        new Regex(@"^\s*@([a-z]+)\s*((!)?\s*([A-Za-z]+))?\s*@\s*$");
    private static readonly Regex OutputDefinitionPattern =
        new Regex(@"\s*@output\s*\((.+)\)\s*@\s*$");

    public void Execute(GeneratorExecutionContext context)
    {
        foreach (var file in context.AdditionalFiles)
        {
            if (file.Path.EndsWith(".t.cs", StringComparison.InvariantCultureIgnoreCase))
            {
                var lines = file.GetText()?.Lines;
                if (lines != null)
                {
                    var (pos, files) = ReadOutputs(lines);
                    foreach (var f in files)
                    {
                        WriteTemplatedFile(context, lines, f, pos - 1);
                    }
                }

            }
        }
    }

    public void Initialize(GeneratorInitializationContext context)
    {

    }

    private (int line, List<OutputFile> files) ReadOutputs(TextLineCollection lines)
    {
        var result = new List<OutputFile>();
        var linenum = 0;
        foreach (var line in lines)
        {
            linenum++;
            var text = line.ToString();
            if (string.IsNullOrWhiteSpace(text))
            {
                continue;
            }

            var match = OutputDefinitionPattern.Match(text);
            if (!match.Success)
            {
                break;
            }

            var args = match.Groups[1];
            var parts = args.Value.Split(new[] { "," }, StringSplitOptions.RemoveEmptyEntries);

            var f = new OutputFile(parts[0]);

            for (var i = 1; i < parts.Length; i++)
            {
                var kvp = parts[i].Split('=');
                if (kvp.Length == 1)
                {
                    f.Flags.Add(kvp[0].Trim());
                }
                else
                {
                    f.Variables[kvp[0].Trim()] = kvp[1].Trim();
                }
            }

            result.Add(f);
        }

        return (linenum, result);
    }

    private void WriteTemplatedFile(GeneratorExecutionContext context,
        TextLineCollection template, OutputFile file, int startLine)
    {
        var ifStarted = false;
        var elseStarted = false;
        var flagValue = false;

        var dest = new StringBuilder();

        for (var linenum = startLine; linenum < template.Count; linenum++)
        {
            var line = template[linenum].ToString();

            var match = StatementPattern.Match(line);
            if (!match.Success)
            {
                if ((!elseStarted && !ifStarted) ||
                    (ifStarted && flagValue) ||
                    (elseStarted && !flagValue))
                {
                    foreach (var variable in file.Variables)
                    {
                        line = line.Replace($"@{variable.Key}@", variable.Value);
                    }

                    dest.AppendLine(line);
                }

                continue;
            }

            var directive = match.Groups[1].Value;
            var notOperator = match.Groups[3].Value;
            var flagName = match.Groups[4].Value;

            if (directive == "if")
            {
                if (ifStarted || elseStarted)
                {
                    dest.AppendLine($"#error Invalid directive '{line}': if statement in previous lines is not closed.");
                    break;
                }

                if (flagName == string.Empty)
                {
                    dest.AppendLine($"#error Invalid directive '{line}': Flag name is not specified.");
                    break;
                }

                ifStarted = true;

                flagValue = file.Flags.IndexOf(flagName) >= 0;
                if (notOperator != string.Empty)
                    flagValue = !flagValue;
            }
            else if (directive == "else")
            {
                if (!ifStarted)
                {
                    dest.AppendLine($"#error Invalid directive '{line}': No if statement in previous lines.");
                    break;
                }

                if (elseStarted)
                {
                    dest.AppendLine($"#error Invalid directive '{line}': else statement in previous lines is not closed .");
                    break;
                }

                ifStarted = false;
                elseStarted = true;
            }
            else if (directive == "endif")
            {
                if (!ifStarted && !elseStarted)
                {
                    dest.AppendLine($"#error Invalid directive '{line}': No if statement in previous lines.");
                    break;
                }

                elseStarted = false;
                ifStarted = false;
            }
            else
            {
                dest.AppendLine($"#error Invalid directive '{line}': Directive '{directive}' is undefined.");
                break;
            }
        }

        context.AddSource(file.FileName, dest.ToString());
    }

    class OutputFile
    {
        public OutputFile(string file)
        {
            FileName = file;
        }

        public string FileName { get; }

        public Dictionary<string, string> Variables { get; } = new Dictionary<string, string>();

        public List<string> Flags { get; } = new List<string>();
    }
}