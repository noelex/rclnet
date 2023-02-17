using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Builders;
using System;
using System.Collections.Generic;
using System.Text;
using System.Xml.Linq;
using Zio;
using Zio.FileSystems;

namespace Rosidl.Generator.CSharp;

record Message(string Package, string SubFolder, string Name, string Path);

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

    public ParseSpec(string specFile)
    {
        if (!File.Exists(specFile))
        {
            throw new Exception($"Spec file '{specFile}' does not exist.");
        }

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
                case "namespace":
                    DefaultRootNamespace = parts[1]; break;
                case "from-ament-index":
                    UseAmentIndex = true; break;
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
                case "map-package":
                    var p = parts[1].Split(':');
                    PackageMapping.Add(p[0], p[1]);
                    break;
                default:
                    throw new Exception($"Unrecognized directive '{directive}'.");
            }
        }
    }
}

public class CSharpCodeGenerator
{
    public static void Generate(string specFilePath)
    {
        var spec = new ParseSpec(specFilePath);
        var packages = new Dictionary<string, Package>();

        var baseDir = Path.GetDirectoryName(Path.GetFullPath(specFilePath))!;
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

        if (spec.UseAmentIndex)
        {
            var path = Environment.GetEnvironmentVariable("AMENT_PREFIX_PATH");
            if (path != null)
            {
                foreach (var p in path.Split(':').Reverse())
                {
                    Console.WriteLine("Searching in directory: " + p);
                    foreach (var pkg in LoadPackages(Path.Combine(p, "share"))) packages[pkg.Name] = pkg;
                }
            }
        }

        foreach (var p in spec.SourceDirectories)
        {
            var path = p;
            if (!Path.IsPathRooted(path))
            {
                path = Path.GetFullPath(Path.Combine(baseDir, path));
            }
            Console.WriteLine("Searching in directory: " + path);
            foreach (var pkg in LoadPackages(path)) packages[pkg.Name] = pkg;
        }

        if (spec.Includes.Count > 0) packages = packages.Where(x => spec.Includes.Contains(x.Key)).ToDictionary(x => x.Key, x => x.Value);
        if (spec.Excludes.Count > 0) packages = packages.Where(x => !spec.Excludes.Contains(x.Key)).ToDictionary(x => x.Key, x => x.Value);

        Console.WriteLine($"Found {packages.Count} package(s), " +
            $"{packages.SelectMany(x => x.Value.Messages).Count()} message definition(s) total.");

        var parser = new MsgParser();
        foreach (var cand in packages.Values)
        {
            foreach (var msg in cand.Messages)
            {
                var metadata = parser.Parse(cand.Name, msg.Name, File.ReadAllText(msg.Path), msg.SubFolder);

                var packageName = opts.ResolvePackageName(cand.Name);
                var dir = Path.Combine(outputDir, packageName);

                CSharpElement code;
                if (metadata is ActionMetadata action)
                {
                    dir = Path.Combine(dir, "Actions");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new ActionBuildContext(action, opts);
                    code = new ActionClassBuilder(ctx)
                        .Build($"{msg.Name}.cs");
                }
                else if (metadata is ServiceMetadata service)
                {
                    dir = Path.Combine(dir, "Services");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new ServiceBuildContext(service, opts);
                    code = new ServiceClassBuilder(ctx)
                        .Build($"{msg.Name}.cs");
                }
                else
                {
                    dir = Path.Combine(dir, "Messages");
                    if (!Directory.Exists(dir))
                        Directory.CreateDirectory(dir);
                    var ctx = new MessageBuildContext((MessageMetadata)metadata, opts);
                    code = new MessageClassBuilder(ctx)
                        .Build($"{msg.Name}.cs");
                }
                var file = (CSharpGeneratedFile)code;

                var pth = Path.Combine(dir, file.FilePath.ToString());
                Console.WriteLine($"Converting {metadata} to {pth}...");

                var fs = new MemoryFileSystem();
                var cw = new CodeWriter(new CodeWriterOptions(fs));
                cw.Options[CSharpGeneratedFile.FileGeneratedByKey] = "ros2cs";
                file.DumpTo(cw);
                var w = fs.ReadAllText("/" + file.FilePath);
                File.WriteAllText(pth, w);
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
                if (root.Element("package")!.Element("member_of_group")!.Value != "rosidl_interface_packages")
                {
                    return false;
                }
            }
            catch
            {
                return false;
            }

            var packageName = Path.GetFileNameWithoutExtension(packageRoot);
            var subdirs = Directory.GetDirectories(packageRoot);
            var results = new List<Message>();
            foreach (var dir in subdirs)
            {
                var type = Path.GetFileName(dir);
                if (type is "msg" or "action" or "srv")
                {
                    results.AddRange(GetMessages(packageName, type, dir));
                }
            }



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

        IEnumerable<Message> GetMessages(string packageName, string subFolder, string path)
        {
            var msgFiles = Directory.GetFiles(path).Where(x => !x.EndsWith(".idl"));
            var results = new List<Message>();
            foreach (var file in msgFiles)
            {
                var msgName = Path.GetFileNameWithoutExtension(file);
                yield return new(packageName, subFolder, msgName, file);
            }
        }
    }
}