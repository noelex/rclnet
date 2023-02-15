// See https://aka.ms/new-console-template for more information
using CppAst;
using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;
using InteropGenerator;
using Zio;
using Zio.FileSystems;

var baseDir = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "../../../../../../modules"));
var rcl = Path.Combine(baseDir, "rcl/rcl/include");
var rcutils = Path.Combine(baseDir, "rcutils/include");
var rmw = Path.Combine(baseDir, "rmw/rmw/include");
var rosidl_runtime_c = Path.Combine(baseDir, "rosidl/rosidl_runtime_c/include");
var rosidl_typesupport = Path.Combine(baseDir, "rosidl/rosidl_typesupport_interface/include");
var yaml = Path.Combine(baseDir, "rcl/rcl_yaml_param_parser/include");


var options = new CSharpConverterOptions();
var temp = options.Plugins.Where(x => x is not DefaultContainerResolver
    and not DefaultTypeConverter
    and not DefaultStructConverter
    and not DefaultFunctionTypeConverter
    and not DefaultTypedefConverter
    and not DefaultFunctionConverter
    and not DefaultDllImportConverter
).ToArray();
options.Plugins.Clear();
options.Plugins.AddRange(temp);
options.Plugins.Add(new CustomContainerResolver());
options.Plugins.Add(new CustomTypeConverter());
options.Plugins.Add(new CustomStructConverter());
options.Plugins.Add(new CustomFunctionTypeConverter());
options.Plugins.Add(new CustomTypedefConverter());
options.Plugins.Add(new CustomFunctionConverter());
options.Plugins.Add(new CustomDllImportConverter());

options.IncludeFolders.Add(rcl);
options.IncludeFolders.Add(rcutils);
options.IncludeFolders.Add(rmw);
options.IncludeFolders.Add(rosidl_runtime_c);
options.IncludeFolders.Add(rosidl_typesupport);
options.IncludeFolders.Add(yaml);

if (OperatingSystem.IsLinux())
{
    options.TargetSystem = "linux";
    options.SystemIncludeFolders.Add("/usr/lib/gcc/x86_64-linux-gnu/9/include/");
}

options.DefaultClassLib = "RclNative";
options.DefaultNamespace = "Rcl.Interop";
options.GenerateAsInternal = true;

options.MappingRules.StandardRules.Add(new CppElementMappingRule(
    new CppElementTypeMatcher(typeof(CppField)),
    new CppElementRegexMatcher("(RCL_DOMAIN_ID_ENV_VAR)|(RCL_DISABLE_LOANED_MESSAGES_ENV_VAR)")).Discard());
options.MappingRules.StandardRules.Add(new CppElementMappingRule(
    new CppElementTypeMatcher(typeof(CppTypedef)),
    new CppElementRegexMatcher("va_list")).Discard());
options.MappingRules.StandardRules.Add(new CppElementMappingRule(
    new CppElementTypeMatcher(typeof(CppFunction)),
    new CppElementRegexMatcher(
    "(rcutils_char_array_vsprintf)|" +
    "(rcutils_snprintf)|" +
    "(rcutils_vsnprintf)|" +
    "(rcutils_log)|" +
    "(rcutils_logging_console_output_handler)")).Discard());


var conv = CSharpConverter.Convert(new List<string> { Path.Combine(rcl, "rcl/rcl.h"), Path.Combine(rcl, "rcl/graph.h") }, options);
if (conv.HasErrors)
{
    foreach (var msg in conv.Diagnostics.Messages)
    {
        Console.WriteLine(msg);
    }
    return;
}

var fs = new MemoryFileSystem();
var cw = new CodeWriter(new(fs));
conv.DumpTo(cw);
var text = fs.ReadAllText(options.DefaultOutputFilePath);

Console.WriteLine(text);