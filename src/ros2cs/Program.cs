// See https://aka.ms/new-console-template for more information
using Rosidl.Generator.CSharp;

if (args.Length == 1 && args[0] is "/?" or "/h" or "-h" or "--help")
{
    Console.WriteLine(
@"
Usage:  ros2cs [SPEC_FILE]

Generates C# source codes from ROS2 interface definitions for use with Rcl.NET.

SPEC_FILE:
    Path to the ros2cs spec file containing configurations for generating source codes.
    If not specified, ros2cs will try to use 'ros2cs.spec' in current directory.
        
For more information, see https://github.com/noelex/rclnet.
        ");
    return;
}

var specFile = args.Length == 0 ? "ros2cs.spec" : args[0];
CSharpCodeGenerator.Generate(specFile);