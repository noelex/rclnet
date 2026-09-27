using CppAst.CodeGen.CSharp;
using CppAst;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace InteropGenerator;


public class CustomDllImportConverter : ICSharpConverterPlugin
{
    /// <inheritdoc />
    public void Register(CSharpConverter converter, CSharpConverterPipeline pipeline)
    {
        pipeline.Converted.Add(AddDefaultDllImport);
    }

    private static readonly Dictionary<string, string> _functionRules=new()
    {
        ["rmw_topic_endpoint_info_array_fini"] = "rmw"
    };

    // Only fixed, short native paths belong here. Allocation, message traversal,
    // middleware calls and callbacks must retain the normal GC transition.
    private static readonly HashSet<string> _suppressGCTransitionFunctions = new(StringComparer.Ordinal)
    {
        "rcutils_get_default_allocator",
        "rcl_get_zero_initialized_arguments",
        "rcl_get_zero_initialized_init_options",
        "rcl_get_zero_initialized_context",
        "rcl_get_zero_initialized_node",
        "rcl_get_zero_initialized_publisher",
        "rcl_get_zero_initialized_subscription",
        "rcl_get_zero_initialized_client",
        "rcl_get_zero_initialized_guard_condition",
        "rcl_get_zero_initialized_service",
        "rcl_get_zero_initialized_timer",
        "rcl_get_zero_initialized_event",
        "rcl_get_zero_initialized_wait_set",
        "rcl_wait_set_add_subscription",
        "rcl_wait_set_add_guard_condition",
        "rcl_wait_set_add_timer",
        "rcl_wait_set_add_client",
        "rcl_wait_set_add_service",
        "rcl_wait_set_add_event",
        "rcl_is_enabled_ros_time_override"
    };

    public static void AddDefaultDllImport(CSharpConverter converter, CSharpElement element, CSharpElement context)
    {
        if (!(element is CSharpMethod method) ||
            (method.Modifiers & CSharpModifiers.Extern) == 0 ||
            method.Attributes.OfType<CSharpDllImportAttribute>().Any())
        {
            return;
        }

        var cppFunction = method.CppElement as CppFunction;
        var callingConvention = cppFunction?.CallingConvention ?? CppCallingConvention.Default;
        var csCallingConvention = callingConvention.GetCSharpCallingConvention();

        string name;
        if(_functionRules.TryGetValue(cppFunction!.Name, out var dll) )
        {
            name = dll;
        }
        else
        {
            name = cppFunction.Name.Split("_", StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)[0];
            if(name == "rmw") name = "rmw_implementation";
        }
        
        method.Attributes.Add(new CSharpDllImportAttribute($"\"{name}\"") { CallingConvention = csCallingConvention });

        if (_suppressGCTransitionFunctions.Contains(cppFunction.Name))
        {
            method.Attributes.Add(new CSharpFreeAttribute("global::System.Runtime.InteropServices.SuppressGCTransitionAttribute"));
        }
    }
}
