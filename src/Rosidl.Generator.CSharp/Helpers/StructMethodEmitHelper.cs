using CppAst.CodeGen.CSharp;
using System.Text;

namespace Rosidl.Generator.CSharp.Helpers;

internal static class StructMethodEmitHelper
{
    public static CSharpMethod EmitThrowIfNonSuccess(this MethodBuildContext context)
    {
        var method = new CSharpMethod()
        {
            Name = "ThrowIfNonSuccess",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Modifiers |= CSharpModifiers.Static;

        method.Parameters.Add(new CSharpParameter("ret") { ParameterType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool), });
        var caller = new CSharpParameter("caller") { ParameterType = new CSharpPrimitiveType(CSharpPrimitiveKind.String), DefaultValue = "\"\"" };
        caller.Attributes.Add(Attributes.CallerMemberName);
        method.Parameters.Add(caller);


        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                if (!ret)
                {
                    throw new global::Rosidl.Runtime.RosidlException($"An error occurred when calling '{{context.StructFullyQualifiedName}}.{caller}'.");
                }
                """);
        };

        return method;
    }

    public static CSharpMethod EmitCopyFromStructRef(this MethodBuildContext context)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "CopyFrom",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("src") { ParameterType = structType.Ref(CSharpRefKind.In) });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                ThrowIfNonSuccess(TryCopy(in src, out this));
                """);
        };

        return method;
    }

    public static CSharpMethod EmitCopyFromStruct(this MethodBuildContext context)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "CopyFrom",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("src") { ParameterType = structType });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                ThrowIfNonSuccess(TryCopy(in src, out this));
                """);
        };

        return method;
    }

    public static CSharpMethod EmitCopyFromPointer(this MethodBuildContext context)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "CopyFrom",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("src") { ParameterType = new CSharpPointerType(structType) });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                fixed ({{structType}}* pThis = &this)
                {
                    ThrowIfNonSuccess(TryCopy(src, pThis));
                }
                """);
        };

        return method;
    }

    public static CSharpFreeMember EmitEquals(this MethodBuildContext context)
    {
        var structType = context.StructType;
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public override bool Equals(object? obj) => obj is {{structType}} s ? this.Equals(s) : false;
            """
        };
    }

    public static CSharpFreeMember EmitStructGetHashCode(this MethodBuildContext context)
    {
        var fields = context.MessageContext.Metadata.Fields
            .OfType<VariableFieldMetadata>().ToArray();

        StringBuilder body = new();
        var idx = 0;
        foreach (var f in fields)
        {
            if (f.Type.TryGetArraySize(out var sz))
            {
                body.AppendLine($"    for (int __i = 0; __i < {sz}; __i++)");
                body.AppendLine("    {");
                body.AppendLine($"        __hashCode.Add(this.{context.MessageContext.GetNormalizedFieldName(f)}[__i]);");
                body.AppendLine("    }");
            }
            else
            {
                body.Append($"    __hashCode.Add(this.{context.MessageContext.GetNormalizedFieldName(f)});");
                if (idx++ < fields.Length - 1)
                {
                    body.AppendLine();
                }
            }
        }

        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public override int GetHashCode()
            {
                var __hashCode = new global::System.HashCode();
            {{body}}
                return __hashCode.ToHashCode();
            }
            """
        };
    }

    public static CSharpFreeMember EmitSequenceGetHashCode(this MethodBuildContext context)
    {
        var fields = context.MessageContext.Metadata.Fields
            .OfType<VariableFieldMetadata>().ToArray();

        StringBuilder body = new();
        var idx = 0;
        foreach (var f in fields)
        {
            body.Append($"    __hashCode.Add(this.{context.MessageContext.GetNormalizedFieldName(f)});");
            if (idx++ < fields.Length - 1)
            {
                body.AppendLine();
            }
        }

        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public override int GetHashCode()
            {
                return global::System.HashCode.Combine((nint)__data, __size, __capacity);
            }
            """
        };
    }

    public static CSharpMethod EmitTypedEquals(this MethodBuildContext context)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "Equals",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new CSharpParameter("other") { ParameterType = structType });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                return {{structType}}.AreEqual(in other, in this);
                """);
        };

        return method;
    }

    public static CSharpFreeMember EmitEqualityOperator(this MethodBuildContext context)
    {
        var structType = context.StructType;
        return new CSharpFreeMember() { Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public static bool operator ==({{structType}} lhs, {{structType}} rhs)
            {
                return lhs.Equals(rhs);
            }
            """ };
    }

    public static CSharpFreeMember EmitInequalityOperator(this MethodBuildContext context)
    {
        var structType = context.StructType;
        return new CSharpFreeMember() { Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public static bool operator !=({{structType}} lhs, {{structType}} rhs)
            {
                return !(lhs == rhs);
            }
            """ };
    }



    public static CSharpFreeMember EmitCopyConstructor(this MethodBuildContext context)
    {
        var type = context.StructType;
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}({{type}} src)
                : this(in src)
            {
            }
            """
        };
    }

    public static CSharpFreeMember EmitCopyConstructorRef(this MethodBuildContext context)
    {
        var type = context.StructType;
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}(in {{type}} src)
                : this()
            {
                CopyFrom(in src); 
            }
            """
        };
    }

    public static CSharpFreeMember EmitCopyConstructorFromPointer(this MethodBuildContext context)
    {
        var type = context.StructType;
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}({{type}}* src)
                : this()
            {
                CopyFrom(src); 
            }
            """
        };
    }

    public static CSharpMethod EmitDispose(this MethodBuildContext context)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "Dispose",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                Finalize(ref this);
                """);
        };

        return method;
    }

    public static CSharpMethod EmitTryCopyRef(this MethodBuildContext context)
    {
        var structType = context.StructType;
        var method = new CSharpMethod()
        {
            Name = "TryCopy",
            Visibility = CSharpVisibility.Private,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new CSharpParameter("input") { ParameterType = context.StructType.Ref(CSharpRefKind.In), });
        method.Parameters.Add(new CSharpParameter("output") { ParameterType = context.StructType.Ref(CSharpRefKind.Out), });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                fixed ({{structType}}* pInput = &input, pOutput = &output)
                {
                    return TryCopy(pInput, pOutput);
                }
                """);
        };

        return method;
    }
}
