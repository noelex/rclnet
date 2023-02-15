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
    }
}