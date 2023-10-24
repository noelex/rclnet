namespace Rosidl.Generator.CSharp;

record CommandlineOption(string Name, string? Value);

class CommandlineOptionParser
{
    record OptionDefinition(string Name, string[] AllowedKeys, bool AllowMultiple = false, bool IsFlag = false);

    record OptionValue(OptionDefinition Definition, string MatchedKey, string? Value);

    private static readonly OptionDefinition[] s_options = new[]
    {
        new OptionDefinition("service-introspection", new[]{ "--service-introspection","--no-service-introspection" }, IsFlag: true),
        new OptionDefinition("action-details", new[]{ "--action-details", "--no-action-details" }, IsFlag: true),
        new OptionDefinition("internal", new[]{ "--internal", "--no-internal" }, IsFlag: true),
        new OptionDefinition("from-ament-index", new[]{ "--from-ament-index", "--no-ament-index" }, IsFlag: true),
        new OptionDefinition("ignore-missing", new[]{ "--ignore-missing", "--no-ignore-missing" }, IsFlag: true),

        new OptionDefinition("namespace", new[]{ "-n", "--namespace" }),
        new OptionDefinition("output", new[]{ "-o", "--output" }),

        new OptionDefinition("from-directory",new[]{ "-s", "--from-directory" }, AllowMultiple: true),
        new OptionDefinition("include",new[]{ "-i", "--include" }, AllowMultiple: true),
        new OptionDefinition("exclude",new[]{ "-e", "--exclude" }, AllowMultiple: true),

        new OptionDefinition("map-namespace",new[]{ "--map-namespace" }, AllowMultiple: true),
        new OptionDefinition("map-name",new[]{ "--map-name" }, AllowMultiple: true),

    };

    private static bool TryMatchOption(string key, out OptionValue? value)
    {
        foreach (var opt in s_options)
        {
            var idx = Array.IndexOf(opt.AllowedKeys, key);
            if (idx >= 0)
            {
                if (opt.IsFlag)
                {
                    value = new OptionValue(opt, opt.AllowedKeys[idx], idx == 0 ? "yes" : "no");
                }
                else
                {
                    value = new OptionValue(opt, opt.AllowedKeys[idx], null);
                }
                return true;
            }

            if (!opt.IsFlag)
            {
                foreach (var k in opt.AllowedKeys)
                {
                    var separator = $"{k}=";
                    if (key.IndexOf(separator) == 0)
                    {
                        value = new OptionValue(opt, k, key[separator.Length..]);
                        return true;
                    }
                }
            }
        }

        value = null;
        return false;
    }

    public static IList<CommandlineOption> Parse(string[] args)
    {
        var options = new List<CommandlineOption>();
        for (var i = 0; i < args.Length; i++)
        {
            if (TryMatchOption(args[i], out var opt))
            {
                if (!opt!.Definition.AllowMultiple && options.Any(x => x.Name == opt.Definition.Name))
                {
                    throw new ArgumentException($"Option '{opt.Definition.Name}' " +
                        $"({string.Join(",", opt.Definition.AllowedKeys)}) can only be specified once.");
                }

                if (!opt!.Definition.IsFlag && string.IsNullOrEmpty(opt.Value))
                {
                    if (i + 1 >= args.Length)
                    {
                        throw new ArgumentException($"Option '{opt.MatchedKey}' requires a value.");
                    }
                    else
                    {
                        options.Add(new(opt.Definition.Name, args[i + 1]));
                        i++;
                    }
                }
                else
                {
                    options.Add(new(opt.Definition.Name, opt.Value));
                }
            }
            else
            {
                if (i == args.Length - 1)
                {
                    // Treat as positional argument.
                    options.Add(new("", args[i]));
                }
                else
                {
                    throw new ArgumentException($"Unknown option '{args[i]}'.");
                }

                break;
            }
        }

        return options;
    }
}