using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Builders;
using System.Xml.Linq;
using Zio;
using Zio.FileSystems;

namespace Rosidl.Generator.CSharp;

record Message(string Package, string SubFolder, string Name, string Path, string? Version)
{
    public ComplexTypeMetadata? Metadata { get; internal set; }
}

record Package(string Name, string Root, Message[] Messages);

class ParseSpec
{
    public Dictionary<string, string> NamespaceMapping { get; } = [];

    public Dictionary<string, string> PackageMapping { get; } = [];

    public bool UseAmentIndex { get; }

    public string DefaultRootNamespace { get; } = "Rosidl.Messages";

    public RosidlAbiMode Abi { get; } = RosidlAbiMode.Portable;

    public string? OutputDirectory { get; }

    public List<string> SourceDirectories { get; } = [];

    public List<string> Includes { get; } = [];

    public List<string> Excludes { get; } = [];

    public bool IsInternal { get; } = false;

    public bool EnableServiceIntrospection { get; } = false;

    public bool EnableActionDetails { get; } = false;

    public string? SpecFile { get; }

    public bool IgnoreMissing { get; } = false;
    internal static readonly char[] s_optSeparator = [' '];

    public ParseSpec(string specFile)
        : this([new("", specFile)])
    {
    }

    public ParseSpec(IEnumerable<CommandlineOption> options)
    {
        var abiSpecified = false;
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
                if (line.StartsWith('#') || line.Length == 0) continue;
                var parts = line.Split([' ', '\t'], StringSplitOptions.RemoveEmptyEntries);
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
                    case "abi":
                        if (parts.Length != 2)
                        {
                            throw new Exception("'abi' requires exactly one argument.");
                        }
                        if (abiSpecified)
                        {
                            throw new Exception("'abi' can only be specified once in a spec file.");
                        }
                        Abi = ParseAbi(parts[1]);
                        abiSpecified = true;
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
                    Includes.AddRange(opt.Value!.Split(s_optSeparator, StringSplitOptions.RemoveEmptyEntries));
                    break;
                case "exclude":
                    Excludes.AddRange(opt.Value!.Split(s_optSeparator, StringSplitOptions.RemoveEmptyEntries));
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
                case "abi":
                    Abi = ParseAbi(opt.Value!);
                    break;
                case "map-package":
                    var p = opt.Value!.Split(':');
                    PackageMapping.Add(p[0], p[1]);
                    break;
            }
        }
    }

    private static RosidlAbiMode ParseAbi(string value)
    {
        return value switch
        {
            "v1" => RosidlAbiMode.V1,
            "v2" => RosidlAbiMode.V2,
            "portable" => RosidlAbiMode.Portable,
            _ => throw new Exception($"'{value}' is not a valid value for 'abi'."),
        };
    }
}

public class CSharpCodeGenerator
{
    public static int Generate(string[] args)
    {
        try
        {
            var opts = CommandlineOptionParser.Parse(args);
            var spec = new ParseSpec(opts);
            var baseDir = spec.SpecFile != null ? Path.GetDirectoryName(Path.GetFullPath(spec.SpecFile))! : Environment.CurrentDirectory;
            return Generate(spec, baseDir,
                string.Equals(opts.FirstOrDefault(o => string.Equals(o.Name, "emit-msbuild-metadata", StringComparison.OrdinalIgnoreCase))?.Value
                        ?? "no", "yes", StringComparison.OrdinalIgnoreCase));
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

    private static int Generate(ParseSpec spec, string baseDir, bool emitMsbuildMetadata = false)
    {
        var packages = new Dictionary<string, Package>();
        var outputDir = spec.OutputDirectory ?? baseDir;
        if (!Path.IsPathRooted(outputDir))
        {
            outputDir = Path.GetFullPath(Path.Combine(baseDir, outputDir));
        }

        var opts = new GeneratorOptions
        {
            RootNamespace = spec.DefaultRootNamespace,
            Abi = spec.Abi
        };
        opts.ResolveNamespace =
            x => spec.NamespaceMapping.TryGetValue(x, out var ns) ? ns : opts.RootNamespace;
        var originalMapper = opts.ResolvePackageName;
        opts.ResolvePackageName =
            x => spec.PackageMapping.TryGetValue(x, out var ns) ? ns : originalMapper(x);

        var sourceRoots = spec.SourceDirectories
            .Select(p => !Path.IsPathRooted(p) ? Path.GetFullPath(Path.Combine(baseDir, p)) : p)
            .ToList();

        var amentRoots = new List<string>();
        if (spec.UseAmentIndex)
        {
            var amentPrefixes = Environment.GetEnvironmentVariable("AMENT_PREFIX_PATH")?
                                           .Split([Path.PathSeparator], StringSplitOptions.RemoveEmptyEntries);
            if (amentPrefixes?.Length > 0)
            {
                amentRoots.AddRange(amentPrefixes.Select(p => Path.Combine(p, "share")));
            }
        }

        var sourcePackageDirectories = sourceRoots
            .SelectMany(EnumeratePackageDirectories)
            .Reverse(); // Reverse so latest found from SourceDirectories takes precedence
        var amentPackageDirectories = amentRoots.SelectMany(EnumeratePackageDirectories);
        var candidatePackageDirectories = sourcePackageDirectories
            .Union(amentPackageDirectories)
            .ToArray();
        var includeAllPackages = spec.Includes.Count == 0;

        var rawPkgs = candidatePackageDirectories
            .Where(ValidPkgDir)
            .Select(p => (Path.GetFileName(p), p))
            .GroupBy(p => p.Item1).Select(g => g.First()) // Only use the latest found package
            .ToDictionary(x => x.Item1, x => x.p);

        if (includeAllPackages) spec.Includes.AddRange(rawPkgs.Select(p => p.Key));

        var inclPkgs = rawPkgs.Where(x => spec.Includes.Contains(x.Key))
                                      .Select(x => (x.Key, TryLoadPackage(x.Value, out var p) ? p : null))
                                      .Where(x => x.Item2 is not null)
                                      .ToDictionary(x => x.Key, x => x.Item2!);

        IEnumerable<string> EnumeratePackageDirectories(string path)
        {
            Console.WriteLine("Searching in directory: " + path);
            return Directory.GetDirectories(path);
        }

        var resolved = new List<string>();
        var unresolved = new List<string>();
        while (true)
        {
            var deps = ResolveDependencies(inclPkgs, spec);
            if (deps.Count == 0 || deps.All(unresolved.Contains)) break;

            foreach (var dep in deps)
            {
                if (rawPkgs.TryGetValue(dep, out var pkgPath)
                    && TryLoadPackage(pkgPath, out var pkg))
                {
                    inclPkgs[dep] = pkg;
                    resolved.Add(dep);
                }
                else
                {
                    unresolved.Add(dep);
                }
            }
        }

        packages = inclPkgs;

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

        var inputsFile = Path.Combine(outputDir, "sources.g.inputs");
        var globsFile = Path.Combine(outputDir, "sources.g.globs");
        var globsPropsFile = Path.Combine(outputDir, "ros2cs.inputs.props");
        var globsStateFile = Path.Combine(outputDir, "ros2cs.globs.state");
        var outputsFile = Path.Combine(outputDir, "sources.g.outputs");
        var previousOutputs = emitMsbuildMetadata && File.Exists(outputsFile)
            ? File.ReadAllLines(outputsFile)
            : [];
        var generatedOutputs = new List<string>();

        if (emitMsbuildMetadata) Directory.CreateDirectory(outputDir);

        var parser = new MsgParser();
        using var fs = new MemoryFileSystem();
        foreach (var cand in packages.Values)
        {
            foreach (var msg in cand.Messages)
            {
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
                else if (metadata is MessageMetadata message)
                {
                    dir = Path.Combine(dir, "Messages");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new MessageBuildContext(message, opts);
                    code = new MessageClassBuilder(ctx)
                        .Build($"{msg.Name}.g.cs", spec.IsInternal);
                }
                else
                {
                    throw new NotSupportedException($"Unsupported interface '{packageName}/{msg.Name}' with type: {metadata?.GetType().FullName ?? "<Unknown>"}");
                }

                var file = (CSharpGeneratedFile)code;

                var genpath = Path.Combine(dir, file.FilePath.ToString());

                var validVer = string.IsNullOrWhiteSpace(metadata.Version) ? null : $" (v{metadata.Version})";
                Console.WriteLine($"Converting {msg.Path}{validVer} to {genpath}");

                if (emitMsbuildMetadata) generatedOutputs.Add(genpath);

                var filePath = "/" + file.FilePath;
                try
                {
                    var cw = new CodeWriter(new CodeWriterOptions(fs));
                    cw.Options[CSharpGeneratedFile.FileGeneratedByKey] = "ros2cs";
                    file.DumpTo(cw);
                    var w = fs.ReadAllLines(filePath);

                    if (File.Exists(genpath) && File.ReadAllLines(genpath).SequenceEqual(w))
                    {
                        // Don't touch the file if unchanged.
                        continue;
                    }

                    File.WriteAllLines(genpath, w);
                }
                finally
                {
                    fs.DeleteFile(filePath);
                }
            }
        }

        if (emitMsbuildMetadata)
        {
            var pathComparer = OperatingSystem.IsWindows()
                ? StringComparer.OrdinalIgnoreCase
                : StringComparer.Ordinal;
            var relevantPackageNames = spec.Includes
                .Union(packages.Keys)
                .Union(missingPackages)
                .ToHashSet(StringComparer.Ordinal);
            var trackedPackageDirectories = includeAllPackages
                ? candidatePackageDirectories
                : candidatePackageDirectories.Where(p => relevantPackageNames.Contains(Path.GetFileName(p)));
            var trackedPackageDirectoryArray = trackedPackageDirectories.ToArray();
            var trackedInputs = trackedPackageDirectoryArray
                .Select(p => Path.Combine(p, "package.xml"))
                .Where(File.Exists)
                .Concat(packages.Values.SelectMany(p => p.Messages.Select(m => m.Path)))
                .Select(Path.GetFullPath)
                .Distinct(pathComparer)
                .Order(pathComparer)
                .ToArray();
            var searchRoots = sourceRoots.Concat(amentRoots).ToArray();
            var packageDiscoveryGlobs = searchRoots.Select(p => Path.Combine(p, "*", "package.xml"));
            var interfaceGlobs = trackedPackageDirectoryArray.SelectMany(p =>
                new[] { "msg", "srv", "action" }.SelectMany(d =>
                    new[] { "*.msg", "*.srv", "*.action" }.Select(f => Path.Combine(p, d, f))));
            var trackedGlobs = packageDiscoveryGlobs
                .Concat(interfaceGlobs)
                .Select(Path.GetFullPath)
                .Distinct(pathComparer)
                .Order(pathComparer)
                .ToArray();
            var globbedInputs = trackedGlobs
                .SelectMany(glob => ExpandGlob(glob, pathComparer))
                .ToArray();
            var currentOutputs = generatedOutputs
                .Select(Path.GetFullPath)
                .Distinct(pathComparer)
                .Order(pathComparer)
                .ToArray();

            foreach (var staleOutput in previousOutputs.Except(currentOutputs, pathComparer))
            {
                if (staleOutput.EndsWith(".g.cs", StringComparison.OrdinalIgnoreCase)
                    && IsPathWithinDirectory(outputDir, staleOutput)
                    && File.Exists(staleOutput))
                {
                    Console.WriteLine("Removing stale generated file: " + staleOutput);
                    File.Delete(staleOutput);
                }
            }

            WriteAllLinesIfChanged(inputsFile, trackedInputs);
            WriteAllLinesIfChanged(globsFile, trackedGlobs);
            WriteGlobsPropsIfChanged(globsPropsFile, trackedGlobs);
            WriteAllLinesIfChanged(globsStateFile, globbedInputs);
            WriteAllLinesIfChanged(outputsFile, currentOutputs);
        }

        PrintStats(false);
        return 0;

        void PrintStats(bool aborted)
        {
            Console.WriteLine();
            Console.WriteLine($"{(aborted ? "Found" : "Processed")} {packages.Count} package(s), " +
                $"{packages.SelectMany(x => x.Value.Messages).Count()} message definition(s) total.");

            if (resolved.Except(spec.Excludes).Any())
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
    }

    private static bool TryLoadPackage(string packageRoot, out Package p)
    {
        p = null!;
        var packageXml = Path.Combine(packageRoot, "package.xml");

        if (!File.Exists(packageXml)) return false;

        XElement pkgXml;
        string packageName;
        try
        {
            pkgXml = XDocument.Load(packageXml).Element("package")!;
            packageName = pkgXml.Element("name")!.Value;
            if (!string.Equals(packageName, Path.GetFileName(packageRoot),
                StringComparison.OrdinalIgnoreCase))
            {
                return false;
            }
        }
        catch
        {
            return false;
        }

        var pkgver = pkgXml.Element("version")?.Value;
        var ver = !string.IsNullOrEmpty(pkgver) ? pkgver : null;

        var results = Directory.GetDirectories(packageRoot)
            .Where(x => Path.GetFileName(x) is "msg" or "action" or "srv")
            .SelectMany(Directory.EnumerateFiles)
            .Where(f => f.EndsWith("msg") || f.EndsWith("action") || f.EndsWith("srv"))
            .Select(msgPath =>
            {
                var file = Path.GetFileNameWithoutExtension(msgPath);
                var subFolder = Path.GetFileName(Path.GetDirectoryName(msgPath))!;
                return new Message(packageName, subFolder, file, msgPath, ver);
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
                var srvName = Path.GetFileName(msg.Path[..endsHere]);
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

        p = new Package(packageName, packageRoot, [.. results]);
        return true;
    }

    private static bool ValidPkgDir(string path)
    {
        var packageXml = Path.Combine(path, "package.xml");
        return File.Exists(packageXml);
    }

    private static bool IsPathWithinDirectory(string directory, string path)
    {
        var relativePath = Path.GetRelativePath(Path.GetFullPath(directory), Path.GetFullPath(path));
        return !Path.IsPathRooted(relativePath)
            && relativePath != ".."
            && !relativePath.StartsWith(".." + Path.DirectorySeparatorChar)
            && !relativePath.StartsWith(".." + Path.AltDirectorySeparatorChar);
    }

    private static void WriteAllLinesIfChanged(string path, IReadOnlyCollection<string> lines)
    {
        if (File.Exists(path) && File.ReadAllLines(path).SequenceEqual(lines)) return;
        File.WriteAllLines(path, lines);
    }

    private static void WriteGlobsPropsIfChanged(string path, IEnumerable<string> globs)
    {
        var document = new XDocument(
            new XElement("Project",
                new XElement("ItemGroup",
                    globs.Select(glob => new XElement("Ros2csManifestGlob", new XAttribute("Include", glob))))));
        var content = document + Environment.NewLine;
        if (File.Exists(path) && File.ReadAllText(path) == content) return;
        File.WriteAllText(path, content);
    }

    private static IEnumerable<string> ExpandGlob(string glob, StringComparer comparer)
    {
        var directory = Path.GetDirectoryName(glob)!;
        var pattern = Path.GetFileName(glob);
        if (Path.GetFileName(directory) == "*")
        {
            var root = Path.GetDirectoryName(directory)!;
            return Directory.Exists(root)
                ? Directory.GetDirectories(root)
                    .Select(p => Path.Combine(p, pattern))
                    .Where(File.Exists)
                    .Order(comparer)
                : [];
        }

        return Directory.Exists(directory)
            ? Directory.GetFiles(directory, pattern).Order(comparer)
            : [];
    }

    private static List<string> ResolveDependencies(Dictionary<string, Package> inclPkgs, ParseSpec spec)
    {
        var missing = new List<string>();

        var parser = new MsgParser();
        foreach (var cand in inclPkgs.Values)
        {
            foreach (var msg in cand.Messages)
            {
                var metadata = parser.Parse(cand.Name, msg.Name, File.ReadAllText(msg.Path), msg.SubFolder);
                metadata.Version = msg.Version;
                msg.Metadata = metadata;

                if (metadata is MessageMetadata m)
                {
                    ResolveFields(inclPkgs, m.Fields, missing);
                }
                else if (metadata is ServiceMetadata s)
                {
                    ResolveFields(inclPkgs, s.RequestFields, missing);
                    ResolveFields(inclPkgs, s.ResponseFields, missing);

                    if (spec.EnableServiceIntrospection)
                    {
                        Requires("service_msgs");
                    }
                }
                else if (metadata is ActionMetadata a)
                {
                    ResolveFields(inclPkgs, a.GoalFields, missing);
                    ResolveFields(inclPkgs, a.ResultFields, missing);
                    ResolveFields(inclPkgs, a.FeedbackFields, missing);

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
            if (!inclPkgs.ContainsKey(package) && !missing.Contains(package))
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
