using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Helpers;

namespace Rosidl.Generator.CSharp.Builders;

public class PrivStructBuilder
{
    public static CSharpElement Build(MessageBuildContext context) => Build(context.NativeLayout);

    internal static CSharpElement Build(NativeLayoutBuildContext context)
    {
        var methodContext = new PrivStructMethodBuildContext(context);
        var structure = new CSharpStruct(context.PrivName);

        structure.AddCommentsForStruct(context.MessageContext.Metadata, context.Layout);

        structure.BaseTypes.Add(new CSharpFreeType($"global::System.IEquatable<{methodContext.StructType}>"));
        structure.BaseTypes.Add(new CSharpFreeType($"global::System.IDisposable"));
        structure.BaseTypes.Add(new CSharpFreeType("global::Rosidl.Runtime.IRosidlNative"));

        structure.Attributes.Add(Attributes.StructLayoutSequential);
        structure.Attributes.Add(Attributes.RosidlAbi(context.Layout));

        var abiProperty = new CSharpProperty("Abi")
        {
            Comment = new XmlComment("<inheritdoc/>"),
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType("global::Rosidl.Runtime.RosidlNativeAbi"),
            GetBodyInlined = context.NativeAbiExpression,
        };
        abiProperty.Attributes.Add(Attributes.DebuggerNonUserCode);
        abiProperty.Attributes.Add(Attributes.GeneratedCode);
        structure.Members.Add(abiProperty);

        var fields = GetFields(context);

        foreach (var variable in fields)
        {
            if (variable.FixedSize != null)
            {
                var arrayType = (ArrayTypeMetadata)variable.Metadata.Type;
                if (arrayType.ElementType is PrimitiveTypeMetadata prim && prim.ValueType is not PrimitiveTypes.String and not PrimitiveTypes.WString)
                {
                    string typeName = context.MessageContext.GetPrimitiveTypeName(prim);
                    structure.Members.Add(
                        new CSharpField("__" + variable.Name + $"[{variable.FixedSize}]")
                        {
                            Visibility = CSharpVisibility.Private,
                            FieldType = new CSharpFreeType((variable.FixedSize != null ? "fixed " : "") + typeName),
                        });
                    var spanProp = new CSharpProperty(variable.Name)
                    {
                        Visibility = CSharpVisibility.Public,
                        ReturnType = new CSharpFreeType($"global::System.Span<{context.MessageContext.GetPrimitiveTypeName(prim)}>")
                    }.AddComments(variable.Metadata);
                    spanProp.GetBody = (writer, element) =>
                    {
                        if (context.RequiresAbiGuard)
                        {
                            writer.WriteLine(context.RequireNativeAbiStatement);
                        }
                        writer.WriteLine($"fixed ({context.GetMessagePrivStructReferenceName(context.MessageContext.Metadata)}* __p = &this) return new (__p->__{variable.Name}, {variable.FixedSize});");
                    };
                    spanProp.Attributes.Add(Attributes.DebuggerNonUserCode);
                    spanProp.Attributes.Add(Attributes.GeneratedCode);
                    structure.Members.Add(spanProp);
                }
                else
                {
                    string typeName;
                    if (arrayType.ElementType is PrimitiveTypeMetadata p)
                    {
                        typeName = p.ValueType is PrimitiveTypes.String
                            ? "global::Rosidl.Runtime.Interop.CString"
                            : "global::Rosidl.Runtime.Interop.U16String";
                    }
                    else
                    {
                        typeName = context.GetMessagePrivStructReferenceName((ComplexTypeMetadata)arrayType.ElementType);
                    }

                    var names = string.Join(", ", Enumerable.Range(0, variable.FixedSize.Value).Select(x => $"__{variable.Name}_{x}"));
                    structure.Members.Add(
                        new CSharpField(names)
                        {
                            Visibility = CSharpVisibility.Private,
                            FieldType = new CSharpFreeType(typeName),
                        });
                    var spanProp = new CSharpProperty(variable.Name)
                    {
                        Visibility = CSharpVisibility.Public,
                        ReturnType = new CSharpFreeType($"global::System.Span<{typeName}>")
                    }.AddComments(variable.Metadata);
                    spanProp.GetBody = (writer, element) =>
                    {
                        if (context.RequiresAbiGuard)
                        {
                            writer.WriteLine(context.RequireNativeAbiStatement);
                        }
                        writer.WriteLine($"fixed ({typeName}* __p = &__{variable.Name}_0) return new (__p, {variable.FixedSize});");
                    };
                    spanProp.Attributes.Add(Attributes.DebuggerNonUserCode);
                    spanProp.Attributes.Add(Attributes.GeneratedCode);
                    structure.Members.Add(spanProp);
                }
            }
            else
            {
                structure.Members.Add(
                    new CSharpField(variable.Name)
                    {
                        Visibility = CSharpVisibility.Public,
                        FieldType = new CSharpFreeType(variable.Type),
                    }.AddComments(variable.Metadata));
            }
        }

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
        var symbolResolver = context.MessageContext.GetNativeMessageFunctionSymbol;
        s.Members.Add(EmitDefaultConstructor(context));
        s.Members.Add(context.EmitCopyConstructor());
        s.Members.Add(context.EmitCopyConstructorRef());
        s.Members.Add(context.EmitCopyConstructorFromPointer());
        s.Members.Add(context.EmitDispose());

        s.Members.Add(context.EmitCopyFromStruct());
        s.Members.Add(context.EmitCopyFromStructRef());
        s.Members.Add(context.EmitCopyFromPointer());
        s.Members.Add(context.EmitTryCopyRef());

        s.Members.Add(context.EmitTypedEquals());
        s.Members.Add(context.EmitEquals());
        s.Members.Add(context.EmitStructGetHashCode());

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

    private static VariableField[] GetFields(NativeLayoutBuildContext context)
    {
        return context.MessageContext.Metadata.Fields
            .OfType<VariableFieldMetadata>()
            .Select(x =>
            {
                var name = context.MessageContext.GetNormalizedFieldName(x);
                var type = GetFieldType(x.Type);
                return new VariableField(name, type, x, x.Type is ArrayTypeMetadata at && !at.IsUpperBounded ? at.Length : null);
            })
            .ToArray();

        string GetFieldType(TypeMetadata type)
        {
            return type switch
            {
                PrimitiveTypeMetadata primitiveType => GetPrimitiveTypeName(primitiveType),
                ArrayTypeMetadata arrayType => GetArrayTypeName(arrayType),
                ComplexTypeMetadata complexType => context.GetMessagePrivStructReferenceName(complexType),
                _ => throw new NotSupportedException(),
            };
        }

        string GetPrimitiveTypeName(PrimitiveTypeMetadata p)
        {
            if (p.ValueType is PrimitiveTypes.String)
            {
                return $"global::Rosidl.Runtime.Interop.CString";
            }
            else if (p.ValueType is PrimitiveTypes.WString)
            {
                return $"global::Rosidl.Runtime.Interop.U16String";
            }
            return context.MessageContext.GetPrimitiveTypeName(p);
        }

        string GetArrayTypeName(ArrayTypeMetadata arrayType)
        {
            if (arrayType.Length != null && !arrayType.IsUpperBounded)
            {
                return arrayType.ElementType switch
                {
                    PrimitiveTypeMetadata primitiveType => GetPrimitiveTypeName(primitiveType),
                    ComplexTypeMetadata complexType => context.GetMessagePrivStructReferenceName(complexType),
                    _ => throw new NotSupportedException(),
                };
            }
            return arrayType.ElementType switch
            {
                PrimitiveTypeMetadata primitiveType => context.GetPrimitiveSequenceTypeName(primitiveType),
                ComplexTypeMetadata complexType => context.GetMessagePrivStructSequenceReferenceName(complexType),
                _ => throw new NotSupportedException(),
            };
        }
    }

    private record VariableField(string Name, string Type, VariableFieldMetadata Metadata, int? FixedSize);

    private static CSharpMethod EmitTryInitialize(MethodBuildContext context, Func<string, string> symbolResolver)
    {
        var structType = context.StructType;
        var method = new CSharpMethod("TryInitialize")
        {
            Visibility = CSharpVisibility.Public,
            Modifiers = CSharpModifiers.Static,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new CSharpParameter("msg") { ParameterType = context.StructType.Ref(CSharpRefKind.Out), });

        method.Body = (writer, element) =>
        {
            if (context.NativeLayoutContext.RequiresAbiGuard)
            {
                writer.WriteLine(context.NativeLayoutContext.RequireNativeAbiStatement);
            }
            writer.WriteLine($$"""
                fixed ({{structType}}* pMsg = &msg)
                {
                    return _PInvoke(pMsg);
                }

                [{{Attributes.SuppressGCTransition}}]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{context.MessageContext.GeneratorLibraryName}}", EntryPoint = "{{symbolResolver("init")}}")]
                static extern {{method.ReturnType}} _PInvoke({{structType}}* msg);
                """);
        };

        return method;
    }

    private static CSharpFreeMember EmitDefaultConstructor(MethodBuildContext context)
    {
        return new CSharpFreeMember()
        {
            Text = $$"""
            [{{Attributes.DebuggerNonUserCode}}]
            [{{Attributes.GeneratedCode}}]
            public {{context.StructName}}()
            {
                ThrowIfNonSuccess(TryInitialize(out this));
            }
            """
        };
    }
}
