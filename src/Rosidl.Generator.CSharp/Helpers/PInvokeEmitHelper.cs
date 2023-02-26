using CppAst.CodeGen.CSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Generator.CSharp.Helpers;

internal static class PInvokeEmitHelper
{
    public static CSharpRefType Ref(this CSharpType type, CSharpRefKind kind) => new CSharpRefType(kind, type);

    public static CSharpMethod EmitFinalize(this MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "Finalize",
            Visibility = CSharpVisibility.Public,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("msg") { ParameterType = context.StructType.Ref(CSharpRefKind.Ref), });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                fixed ({{structType}}* pMsg = &msg)
                {
                    _PInvoke(pMsg);
                }

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("fini")}}")]
                static extern {{method.ReturnType}} _PInvoke({{structType}}* msg);
                """);
        };

        return method;
    }

    public static CSharpMethod EmitTryCopy(this MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "TryCopy",
            Visibility = CSharpVisibility.Private,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new CSharpParameter("input") { ParameterType = new CSharpPointerType( context.StructType), });
        method.Parameters.Add(new CSharpParameter("output") { ParameterType = new CSharpPointerType(context.StructType), });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                return _PInvoke(input, output);

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("copy")}}")]
                static extern {{method.ReturnType}} _PInvoke({{structType}}* input, {{structType}}* output);
                """);
        };

        return method;
    }

    public static CSharpMethod EmitAreEqual(this MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "AreEqual",
            Visibility = CSharpVisibility.Private,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new CSharpParameter("lhs") { ParameterType = context.StructType.Ref(CSharpRefKind.In), });
        method.Parameters.Add(new CSharpParameter("rhs") { ParameterType = context.StructType.Ref(CSharpRefKind.In), });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                fixed ({{structType}}* plhs = &lhs, prhs = &rhs)
                {
                    return _PInvoke(plhs, prhs);
                }

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("are_qual")}}")]
                static extern {{method.ReturnType}} _PInvoke({{structType}}* lhs, {{structType}}* rhs);
                """);
        };

        return method;
    }

    public static CSharpMethod EmitCreate(this MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "Create",
            Visibility = CSharpVisibility.Public,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPointerType(context.StructType)
        };

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                return _PInvoke();

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("create")}}")]
                static extern {{method.ReturnType}} _PInvoke();
                """);
        };

        return method;
    }

    public static CSharpMethod EmitDestroy(this MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "Destroy",
            Visibility = CSharpVisibility.Public,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("msg") { ParameterType = new CSharpPointerType(context.StructType) });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                _PInvoke(msg);

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("destroy")}}")]
                static extern {{method.ReturnType}} _PInvoke({{structType}}* msg);
                """);
        };

        return method;
    }
}
