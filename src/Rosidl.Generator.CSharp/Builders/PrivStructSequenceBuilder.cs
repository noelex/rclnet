using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Helpers;

namespace Rosidl.Generator.CSharp.Builders;

public class PrivStructSequenceBuilder
{
    public static CSharpElement Build(MessageBuildContext context)
    {
        var methodContext = new SequenceStructMethodBuildContext(context);
        var structure = new CSharpStruct(context.PrivStructSequenceName);

        structure.AddCommentsForStructSequence(context.Metadata);

        structure.BaseTypes.Add(new CSharpFreeType($"global::System.IEquatable<{methodContext.StructType}>"));
        structure.BaseTypes.Add(new CSharpFreeType($"global::System.IDisposable"));

        structure.Attributes.Add(Attributes.StructLayoutSequential);



        structure.Members.Add(new CSharpField("__data")
        {
            Visibility = CSharpVisibility.Private,
            FieldType = new CSharpPointerType(new CSharpFreeType(context.PrivStructName)),
        });

        structure.Members.Add(new CSharpField("__size")
        {
            Visibility = CSharpVisibility.Private,
            FieldType = new CSharpFreeType("nuint"),
        });

        structure.Members.Add(new CSharpField("__capacity")
        {
            Visibility = CSharpVisibility.Private,
            FieldType = new CSharpFreeType("nuint"),
        });

        structure.Members.Add(new CSharpProperty("Size")
        {
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Int),
            GetBodyInlined = "(int)__size"
        });

        structure.Members.Add(new CSharpProperty("Capcacity")
        {
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Int),
            GetBodyInlined = "(int)__capacity"
        });

        EmitMethods(structure, methodContext);

        foreach (var m in structure.Members.OfType<CSharpMethod>())
        {
            m.Attributes.Add(Attributes.DebuggerNonUserCode);
            m.Attributes.Add(Attributes.GeneratedCode);
        }

        return structure;
    }

    private static void EmitMethods(CSharpStruct s, MethodBuildContext context)
    {
        var symbolResolver = context.MessageContext.GetNativeSequenceFunctionSymbol;
        s.Members.Add(EmitDefaultConstructor(context));
        s.Members.Add(EmitConstructorWithSize(context));
        s.Members.Add(context.EmitCopyConstructor());
        s.Members.Add(context.EmitCopyConstructorRef());
        s.Members.Add(context.EmitCopyConstructorFromPointer());
        s.Members.Add(EmitCopyConstructorSpan(context));
        s.Members.Add(context.EmitDispose());

        s.Members.Add(EmitAsSpan(context));
        s.Members.Add(EmitCopyFromSpan(context));
        s.Members.Add(context.EmitCopyFromStruct());
        s.Members.Add(context.EmitCopyFromStructRef());
        s.Members.Add(context.EmitCopyFromPointer());
        s.Members.Add(context.EmitTryCopyRef());

        s.Members.Add(context.EmitTypedEquals());
        s.Members.Add(context.EmitEquals());
        s.Members.Add(context.EmitSequenceGetHashCode());

        s.Members.Add(context.EmitEqualityOperator());
        s.Members.Add(context.EmitInequalityOperator());
        s.Members.Add(context.EmitCreate(symbolResolver));
        s.Members.Add(context.EmitDestroy(symbolResolver));

        s.Members.Add(EmitTryInitialize(context, symbolResolver));
        s.Members.Add(context.EmitFinalize(symbolResolver));
        s.Members.Add(context.EmitAreEqual(symbolResolver));
        s.Members.Add(context.EmitTryCopy(symbolResolver));

        s.Members.Add(context.EmitThrowIfNonSuccess());
    }

    private static CSharpMethod EmitTryInitialize(MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod("TryInitialize")
        {
            Visibility = CSharpVisibility.Public,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new CSharpParameter("size") { ParameterType = new CSharpPrimitiveType(CSharpPrimitiveKind.Int), });
        method.Parameters.Add(new CSharpParameter("msg") { ParameterType = context.StructType.Ref(CSharpRefKind.Out), });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                fixed ({{structType}}* pMsg = &msg)
                {
                    return _PInvoke(pMsg, (uint)size);
                }

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("init")}}")]
                static extern {{method.ReturnType}} _PInvoke({{structType}}* msg, nuint size);
                """);
        };

        return method;
    }

    private static CSharpMethod EmitAsSpan(MethodBuildContext context)
    {
        return new CSharpMethod("AsSpan")
        {
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType($"System.Span<{context.MessageContext.PrivStructName}>"),
            Body = (writer, element) =>
            {
                writer.WriteLine("return new(__data, Size);");
            }
        };
    }

    private static CSharpMethod EmitCopyFromSpan(MethodBuildContext context)
    {
        var method = new CSharpMethod("CopyFrom")
        {
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("src")
        {
            ParameterType = new CSharpFreeType($"System.ReadOnlySpan<{context.MessageContext.PrivStructName}>")
        });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                    Finalize(ref this);
                    ThrowIfNonSuccess(TryInitialize(src.Length, out this));
                    src.CopyTo(AsSpan());
                    """);
        };

        return method;
    }

    private static CSharpFreeMember EmitCopyConstructorSpan(MethodBuildContext context)
    {
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}(System.ReadOnlySpan<{{context.MessageContext.PrivStructName}}> src)
                : this(src.Length)
            {
                src.CopyTo(AsSpan());
            }
            """
        };
    }

    private static CSharpFreeMember EmitDefaultConstructor(MethodBuildContext context)
    {
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}()
                : this(0)
            {
            }
            """
        };
    }

    private static CSharpFreeMember EmitConstructorWithSize(MethodBuildContext context)
    {
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}(int size)
            {
                ThrowIfNonSuccess(TryInitialize(size, out this));
            }
            """
        };
    }
}