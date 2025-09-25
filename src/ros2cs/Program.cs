// See https://aka.ms/new-console-template for more information
using Rosidl.Generator.CSharp;
using System.Diagnostics;
using System.Reflection;

if (args.Length == 1 && args[0] is "-v" or "--version")
{
    Console.WriteLine(typeof(Program).Assembly
        .GetCustomAttribute<AssemblyInformationalVersionAttribute>()!.InformationalVersion);
    return 0;
}

if (args.Length == 1 && args[0] is "/?" or "/h" or "-h" or "--help")
{
    Console.WriteLine(
@"
Usage:  ros2cs <OPTIONS> <SPEC_FILE>

Generates C# source code from ROS2 interface definitions for use with Rcl.NET.

OPTIONS:
    --service-introspection,
    --no-service-introspection
        Generate code for service introspection.

        This feature requires 'service_msgs' package.
        This option is disabled by default.

    --action-details,
    --no-action-details
        Generate implementation details for actions, including FeedbackMessage, SendGoal 
        request/response and GetResult request/response.
        Detail messages are not neccesary for using action servers and clients as rclnet
        utilizes introspection to construct and extract Feedback / Goal and Result messages.
        Enabling 'action-details' will allow you to perform service introspection on Result
        and Goal services, or manually subscribe Feedback messages, at the cost of increased
        code size.

        This feature requires 'unique_identifier_msgs' and 'builtin_interfaces' package.
        This option is disabled by default.

    --internal,
    --no-internal
        Generate classes with internal visibility rather than public.
        
        This option is disabled by default. 
    
    --from-ament-index,
    --no-ament-index
        Read packages from ament prefix directories as specified by AMENT_PREFIX_PATH.
        If this flag specified, ament prefix directories always load before all other
        'from-directory' directives.

        This option is disabled by default.

    --ignore-missing,
    --no-ignore-missing
        Ignore missing included/depedency packages, proceed with packages currently available
        anyway.

        By default, ros2cs will abort with exit code 1 if any package dependency is missing, 
        unless missing packages are excluded explicitly using 'exclude' option.
        Enabling this option forces ros2cs to generate code even if some packages are missing.

        This option is disabled by default.

    -df,
    --details-file
        Generate a details file 'sources.g.inputs' and 'generated.g.outputs' containing paths
        of all input packages and generated files.
        
        This option is disabled by default.

    -o,
    --output=OUTPUT_DIR
        Specifiy the output directory.
        A path relative to the SPEC_FILE (if not specified, relative to current directory),
        or an absolute path can be used.

    -I,
    --from-directory=INCLUDE_DIR
        Read packages from specified root package directory containing one or more packages.
        A path relative to this file or an absolute path can be used.

        This option may be specified multiple times.

    -n,
    --namespace=NAMESPACE
        Set the default root namespace of the generated code.

    -i,
    --include=""PKG1 PKG2 ...""
        Generate codes for specified packages only.
        Dependencies of the specified packages will also be included recursively.
        If no include directive is specified, ros2cs will generate codes for all discovered
        packages.

        This option may be specified multiple times.

    -e,
    --exclude=""PKG1 PKG2 ...""
        Explicitly exclude packages from code generation.

        This option may be specified multiple times.

    --map-namespace=PKG:NAMESPACE
        Set the root namespace of a specific package,
        e.g. '--map-name my_messages:My.Namespace' will map definitions in package 'my_messages'
        into C# namespace My.Namespace.MyMessages.
        If not specified, default root namespace is used.

        This option may be specified multiple times.

    --map-name=PKG:NAME
        Set the name of a specific package,
        e.g. 'map-name my_messages:Messages' will map definitions in package 'my_messages'
        into C# namespace Rosidl.Messages.Messages.
        If not specified, ros2cs will determine the package name automatically.

        This option may be specified multiple times.

    -v,
    --version
        Print the version of this program.

    -h,
    --help
        Print this message.

SPEC_FILE:
    Path to a ros2cs spec file containing configurations for generating source code.

    If neither OPTIONS nor SPEC_FILE is specified, ros2cs will try to use 'ros2cs.spec'
    in current directory as SPEC_FILE.
    
    If both OPTIONS and SPEC_FILE are specified, ros2cs will load configurations from
    SPEC_FILE first, and override configurations with options specified by OPTIONS.
        
For more information, see https://github.com/noelex/rclnet.");
    return 0;
}

return CSharpCodeGenerator.Generate(args);