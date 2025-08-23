using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Builders;
using System.IO;
using System.Xml.Linq;
using Zio;
using Zio.FileSystems;

namespace Rosidl.Generator.CSharp;

record Message(string Package, string SubFolder, string Name, string Path)
{
    public ComplexTypeMetadata? Metadata { get; internal set; }
}

record Package(string Name, Message[] Messages);

class ParseSpec
{
    public Dictionary<string, string> NamespaceMapping { get; } = new();

    public Dictionary<string, string> PackageMapping { get; } = new();

    public bool UseAmentIndex { get; }

    public string DefaultRootNamespace { get; } = "Rosidl.Messages";

    public string? OutputDirectory { get; }

    public List<string> SourceDirectories { get; } = new();

    public List<string> Includes { get; } = new();

    public List<string> Excludes { get; } = new();

    public bool IsInternal { get; } = false;

    public bool EnableServiceIntrospection { get; } = false;

    public bool EnableActionDetails { get; } = false;

    public string? SpecFile { get; }

    public bool IgnoreMissing { get; } = false;

    public ParseSpec(string specFile)
        : this(new CommandlineOption[] { new("", specFile) })
    {
    }

    public ParseSpec(IEnumerable<CommandlineOption> options)
    {
        var specFile = options.FirstOrDefault(x => x.Name == string.Empty)?.Value;
        if (specFile == null && !options.Any())
        {
            specFile = Path.Combine(Environment.CurrentDirectory, "ros2cs.spec");
        }

        if (specFile != null)
        {
            if (!File.Exists(specFile))
            {
                throw new Exception($"Spec file '{specFile}' does not exist.");
            }

            SpecFile = specFile;
            var lines = File.ReadAllLines(specFile);
            foreach (var line in lines.Select(x => x.Trim()))
            {
                if (line.StartsWith("#") || line.Length == 0) continue;
                var parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                var directive = parts[0];
                switch (directive)
                {
                    case "output":
                        OutputDirectory = parts[1]; break;
                    case "internal":
                        IsInternal = true; break;
                    case "namespace":
                        DefaultRootNamespace = parts[1]; break;
                    case "from-ament-index":
                        UseAmentIndex = true; break;
                    case "ignore-missing":
                        IgnoreMissing = true; break;
                    case "from-directory":
                        SourceDirectories.Add(parts[1]);
                        break;
                    case "include":
                        Includes.AddRange(parts[1..]);
                        break;
                    case "exclude":
                        Excludes.AddRange(parts[1..]);
                        break;
                    case "map-namespace":
                        var mapping = parts[1].Split(':');
                        NamespaceMapping.Add(mapping[0], mapping[1]);
                        break;
                    case "service-introspection":
                        if (parts.Length != 2)
                        {
                            throw new Exception($"'service-introspection' requires exactly one argument.");
                        }
                        EnableServiceIntrospection = parts[1] switch
                        {
                            "on" => true,
                            "off" => false,
                            _ => throw new Exception($"'{parts[1]}' is not a valid value for 'service-introspection' directive."),
                        };
                        break;
                    case "action-details":
                        if (parts.Length != 2)
                        {
                            throw new Exception($"'action-details' requires exactly one argument.");
                        }
                        EnableActionDetails = parts[1] switch
                        {
                            "on" => true,
                            "off" => false,
                            _ => throw new Exception($"'{parts[1]}' is not a valid value for 'action-details' directive."),
                        };
                        break;
                    case "map-package":
                        var p = parts[1].Split(':');
                        PackageMapping.Add(p[0], p[1]);
                        break;
                    default:
                        throw new Exception($"Unrecognized directive '{directive}'.");
                }
            }
        }

        foreach (var opt in options)
        {
            switch (opt.Name)
            {
                case "output":
                    OutputDirectory = opt.Value; break;
                case "internal":
                    IsInternal = opt.Value == "yes"; break;
                case "namespace":
                    DefaultRootNamespace = opt.Value!; break;
                case "from-ament-index":
                    UseAmentIndex = opt.Value == "yes"; break;
                case "ignore-missing":
                    IgnoreMissing = opt.Value == "yes"; break;
                case "from-directory":
                    SourceDirectories.Add(opt.Value!);
                    break;
                case "include":
                    Includes.AddRange(opt.Value!.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries));
                    break;
                case "exclude":
                    Excludes.AddRange(opt.Value!.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries));
                    break;
                case "map-namespace":
                    var mapping = opt.Value!.Split(':');
                    NamespaceMapping.Add(mapping[0], mapping[1]);
                    break;
                case "service-introspection":
                    EnableServiceIntrospection = opt.Value == "yes";
                    break;
                case "action-details":
                    EnableActionDetails = opt.Value == "yes";
                    break;
                case "map-package":
                    var p = opt.Value!.Split(':');
                    PackageMapping.Add(p[0], p[1]);
                    break;
            }
        }
    }
}

public class CSharpCodeGenerator
{
    public static int Generate(string[] args)
    {
        try
        {
            var spec = new ParseSpec(CommandlineOptionParser.Parse(args));
            var baseDir = spec.SpecFile != null ? Path.GetDirectoryName(Path.GetFullPath(spec.SpecFile))! : Environment.CurrentDirectory;
            return Generate(spec, baseDir);
        }
        catch (Exception ex)
        {
            Console.WriteLine(ex.Message);
            return 2;
        }
    }

    public static void Generate(string specFilePath)
    {
        var spec = new ParseSpec(specFilePath);
        var baseDir = Path.GetDirectoryName(Path.GetFullPath(specFilePath))!;
        Generate(spec, baseDir);
    }

    private static int Generate(ParseSpec spec, string baseDir)
    {
        var packages = new Dictionary<string, Package>();
        var outputDir = spec.OutputDirectory ?? baseDir;
        if (!Path.IsPathRooted(outputDir))
        {
            outputDir = Path.GetFullPath(Path.Combine(baseDir, outputDir));
        }

        var opts = new GeneratorOptions();
        opts.RootNamespace = spec.DefaultRootNamespace;
        opts.ResolveNamespace =
            x => spec.NamespaceMapping.TryGetValue(x, out var ns) ? ns : opts.RootNamespace;
        var originalMapper = opts.ResolvePackageName;
        opts.ResolvePackageName =
            x => spec.PackageMapping.TryGetValue(x, out var ns) ? ns : originalMapper(x);

        IEnumerable<Package> pkgs = [];
        if (spec.UseAmentIndex)
        {
            var amentPrefixes = Environment.GetEnvironmentVariable("AMENT_PREFIX_PATH");
            if (!string.IsNullOrWhiteSpace(amentPrefixes))
            {
                pkgs = amentPrefixes.Split([Path.PathSeparator], StringSplitOptions.RemoveEmptyEntries)
                    .SelectMany(p =>
                    {
                        Console.WriteLine("Searching in directory: " + p);
                        return LoadPackages(Path.Combine(p, "share"));
                    });
            }
        }

        packages = spec.SourceDirectories
            .Select(p => !Path.IsPathRooted(p) ? Path.GetFullPath(Path.Combine(baseDir, p)) : p)
            .SelectMany(p =>
            {
                Console.WriteLine("Searching in directory: " + p);
                return LoadPackages(p);
            })
            .Union(pkgs)
            .GroupBy(p => p.Name).Select(g => g.First()) // Take the first package with the same name
            .ToDictionary(p => p.Name, p => p);

        if (spec.Includes.Count == 0) spec.Includes.AddRange(packages.Keys);

        var includedPackages = packages.Where(x => spec.Includes.Contains(x.Key)).ToDictionary(x => x.Key, x => x.Value);
        var resolved = new List<string>();
        var unresolved = new List<string>();
        while (true)
        {
            var deps = ResolveDependencies(includedPackages, spec);
            if (deps.Count == 0 || deps.All(unresolved.Contains)) break;

            foreach (var dep in deps)
            {
                if (packages.TryGetValue(dep, out var pkg))
                {
                    includedPackages[dep] = pkg;
                    resolved.Add(dep);
                }
                else
                {
                    unresolved.Add(dep);
                }
            }
        }

        packages = includedPackages;

        if (spec.Excludes.Count > 0)
        {
            packages = packages.Where(x => !spec.Excludes.Contains(x.Key)).ToDictionary(x => x.Key, x => x.Value);
        }

        var missingPackages = unresolved
            .Union(spec.Includes.Except(packages.Keys))
            .Except(spec.Excludes)
            .Distinct()
            .ToArray();
        if (missingPackages.Length > 0 && !spec.IgnoreMissing)
        {
            PrintStats(true);
            return 1;
        }

        var parser = new MsgParser();
        using var fs = new MemoryFileSystem();
        foreach (var cand in packages.Values)
        {
            foreach (var msg in cand.Messages)
            {
                //var metadata = parser.Parse(cand.Name, msg.Name, File.ReadAllText(msg.Path), msg.SubFolder);
                var metadata = msg.Metadata;

                var packageName = opts.ResolvePackageName(cand.Name);
                var dir = Path.Combine(outputDir, packageName);

                CSharpElement code;
                if (metadata is ActionMetadata action)
                {
                    dir = Path.Combine(dir, "Actions");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new ActionBuildContext(action, opts, parser);
                    code = new ActionClassBuilder(ctx)
                        .Build($"{msg.Name}.g.cs", spec.IsInternal, spec.EnableServiceIntrospection, spec.EnableActionDetails);
                }
                else if (metadata is ServiceMetadata service)
                {
                    dir = Path.Combine(dir, "Services");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new ServiceBuildContext(service, opts);
                    code = new ServiceClassBuilder(ctx)
                        .Build($"{msg.Name}.g.cs", spec.IsInternal, spec.EnableServiceIntrospection);
                }
                else
                {
                    dir = Path.Combine(dir, "Messages");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new MessageBuildContext((MessageMetadata)metadata, opts);
                    code = new MessageClassBuilder(ctx)
                        .Build($"{msg.Name}.g.cs", spec.IsInternal);
                }
                var file = (CSharpGeneratedFile)code;

                var pth = Path.Combine(dir, file.FilePath.ToString());
                Console.WriteLine($"Converting {metadata} to {pth} ...");

                var filePath = "/" + file.FilePath;
                try
                {
                    var cw = new CodeWriter(new CodeWriterOptions(fs));
                    cw.Options[CSharpGeneratedFile.FileGeneratedByKey] = "ros2cs";
                    file.DumpTo(cw);
                    var w = fs.ReadAllLines(filePath);

                    if (File.Exists(pth) && File.ReadAllLines(pth).SequenceEqual(w))
                    {
                        // Don't touch the file if unchanged.
                        continue;
                    }

                    File.WriteAllLines(pth, w);
                }
                finally
                {
                    fs.DeleteFile(filePath);
                }
            }
        }

        PrintStats(false);
        return 0;

        void PrintStats(bool aborted)
        {
            Console.WriteLine();
            Console.WriteLine($"{(aborted ? "Found" : "Processed")} {packages.Count} package(s), " +
                $"{packages.SelectMany(x => x.Value.Messages).Count()} message definition(s) total.");

            if (resolved.Except(spec.Excludes).Count() > 0)
            {
                Console.WriteLine();
                Console.WriteLine("The following package(s) were automatically included as dependencies:");
                foreach (var dep in resolved)
                {
                    if (!spec.Excludes.Contains(dep)) Console.WriteLine(dep);
                }
            }

            if (missingPackages.Length > 0)
            {
                Console.WriteLine();
                Console.WriteLine("The following package(s) were not found in configured package sources:");
                foreach (var dep in missingPackages)
                {
                    Console.WriteLine(dep);
                }
            }

            if (aborted)
            {
                Console.WriteLine();
                Console.WriteLine("Source generation is aborted due to missing package(s).");
            }
        }

        IEnumerable<Package> LoadPackages(string directory)
        {
            foreach (var dir in Directory.GetDirectories(directory))
            {
                if (TryLoadPackage(dir, out var p))
                {
                    yield return p;
                }
            }
        }

        bool TryLoadPackage(string packageRoot, out Package p)
        {
            p = null!;
            var packageXml = Path.Combine(packageRoot, "package.xml");

            if (!File.Exists(packageXml)) return false;

            try
            {
                var root = XDocument.Load(packageXml);
                if (root.Element("package")!.Element("name")?.Value != Path.GetFileName(packageRoot))
                {
                    return false;
                }
            }
            catch
            {
                return false;
            }

            var packageName = Path.GetFileNameWithoutExtension(packageRoot);
            var results = Directory.GetDirectories(packageRoot)
                .Where(x => Path.GetFileName(x) is "msg" or "action" or "srv")
                .SelectMany(Directory.EnumerateFiles)
                .Where(f => f.EndsWith("msg") || f.EndsWith("action") || f.EndsWith("srv"))
                .Select(p =>
                { 
                    var file = Path.GetFileNameWithoutExtension(p);
                    var subFolder = Path.GetFileName(Path.GetDirectoryName(p));
                    return new Message(packageName, subFolder, file, p);
                })
                .ToList();

            var duplicates = new List<Message>();
            foreach (var msg in results)
            {
                if (msg.SubFolder == "srv")
                {
                    if (msg.Path.EndsWith("_Response.msg"))
                    {
                        Dedup("_Response.msg");
                    }
                    else if (msg.Path.EndsWith("_Request.msg"))
                    {
                        Dedup("_Request.msg");
                    }
                }

                void Dedup(string postfix)
                {
                    var endsHere = msg.Path.LastIndexOf(postfix);
                    var srvName = Path.GetFileName(msg.Path.Substring(0, endsHere));
                    if (results.Any(x => x.Name == srvName))
                    {
                        duplicates.Add(msg);
                    }
                }
            }

            foreach (var dup in duplicates)
            {
                results.Remove(dup);
            }

            if (results.Count == 0) return false;

            p = new Package(packageName, results.ToArray());
            return true;
        }
    }

    private static List<string> ResolveDependencies(Dictionary<string, Package> packages, ParseSpec spec)
    {
        var missing = new List<string>();

        var parser = new MsgParser();
        foreach (var cand in packages.Values)
        {
            foreach (var msg in cand.Messages)
            {
                var metadata = parser.Parse(cand.Name, msg.Name, File.ReadAllText(msg.Path), msg.SubFolder);
                msg.Metadata = metadata;
                if (metadata is MessageMetadata m)
                {
                    ResolveFields(packages, m.Fields, missing);
                }
                else if (metadata is ServiceMetadata s)
                {
                    ResolveFields(packages, s.RequestFields, missing);
                    ResolveFields(packages, s.ResponseFields, missing);

                    if (spec.EnableServiceIntrospection)
                    {
                        Requires("service_msgs");
                    }
                }
                else if (metadata is ActionMetadata a)
                {
                    ResolveFields(packages, a.GoalFields, missing);
                    ResolveFields(packages, a.ResultFields, missing);
                    ResolveFields(packages, a.FeedbackFields, missing);

                    if (spec.EnableServiceIntrospection)
                    {
                        Requires("service_msgs");
                    }

                    if (spec.EnableActionDetails)
                    {
                        Requires("builtin_interfaces");
                        Requires("unique_identifier_msgs");
                    }
                }
            }
        }

        return missing;

        void Requires(string package)
        {
            if (!packages.ContainsKey(package) && !missing.Contains(package))
            {
                missing.Add(package);
            }
        }
    }

    private static void ResolveFields(Dictionary<string, Package> packages, IEnumerable<FieldMetadata> fields, List<string> missingDependencies)
    {
        foreach (var f in fields)
        {
            if (f.Type is ComplexTypeMetadata cm &&
                !packages.ContainsKey(cm.Id.Package) &&
                !missingDependencies.Contains(cm.Id.Package))
            {
                missingDependencies.Add(cm.Id.Package);
            }
        }
    }
}