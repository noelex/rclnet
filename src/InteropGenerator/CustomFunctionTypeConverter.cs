using CppAst.CodeGen.CSharp;
using CppAst;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteropGenerator;

public class CustomFunctionTypeConverter : ICSharpConverterPlugin
{
    /// <inheritdoc />
    public void Register(CSharpConverter converter, CSharpConverterPipeline pipeline)
    {
        pipeline.FunctionTypeConverters.Add(ConvertAnonymousFunctionType);
    }

    public static bool IsFunctionType(CppType type, out CppFunctionType cppFunctionType)
    {
        type = type.GetCanonicalType();
        cppFunctionType = (type as CppFunctionType)!;

        if (cppFunctionType == null)
        {
            if (type is CppPointerType ptrType && (ptrType.ElementType is CppFunctionType cppFunctionType2))
            {
                cppFunctionType = cppFunctionType2;
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    public static CSharpElement ConvertAnonymousFunctionType(CSharpConverter converter, CppFunctionType cppFunctionType, CSharpElement context)
    {
        return ConvertNamedFunctionType(converter, cppFunctionType, context, null!);
    }

    public static CSharpType ConvertNamedFunctionType(CSharpConverter converter, CppFunctionType cppType, CSharpElement context, CppTypedef typedef)
    {
        if (cppType == null) throw new ArgumentNullException(nameof(cppType));

        
        //string name = typedef?.Name;

        //if (typedef == null)
        //{
        //    name = converter.GetCSharpName(cppType, context);
        //}

        //var csDelegate = new CSharpDelegate(name) { CppElement = cppType };
        var cppFunctionType = cppType;

        var fp = new CSharpFunctionPointer(converter.GetCSharpType(cppFunctionType.ReturnType, context)) { CppElement = cppType, IsUnmanaged = true };

        fp.Parameters.AddRange(cppFunctionType.Parameters.Select(x => converter.GetCSharpType(x.Type, context)));

        // Add calling convention
        var csCallingConvention = cppFunctionType.CallingConvention.GetCSharpCallingConvention();
        fp.UnmanagedCallingConvention.Add(csCallingConvention.ToString());
        //fp.UnmanagedCallingConvention.Add("SuppressGCTransition");
        // csDelegate.Attributes.Add(new CSharpFreeAttribute($"UnmanagedFunctionPointer(CallingConvention.{csCallingConvention})"));

        var container = typedef != null
            ? converter.GetCSharpContainer(typedef, context)
            : converter.GetCSharpContainer(cppFunctionType, context);

        //if (container is CSharpInterface)
        //{
        //    container = container.Parent;
        //}

        //converter.ApplyDefaultVisibility(csDelegate, container);
        //container.Members.Add(csDelegate);

        converter.AddUsing(container, "System.Runtime.InteropServices");

        //csDelegate.Comment = converter.GetCSharpComment(cppFunctionType, csDelegate);
        //csDelegate.ReturnType = converter.GetCSharpType(cppFunctionType.ReturnType, csDelegate);

        return fp;
    }
}