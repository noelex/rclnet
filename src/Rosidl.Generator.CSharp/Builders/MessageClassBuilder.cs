using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;
using Rosidl.Generator.CSharp.Helpers;
using System.Text;

namespace Rosidl.Generator.CSharp.Builders;

public class MessageClassBuilder
{
    private readonly MessageBuildContext _context;
    private readonly VariableFieldInfo[] _variables;
    public MessageClassBuilder(MessageBuildContext context)
    {
        _context = context;
        _variables = context.Metadata.Fields.OfType<VariableFieldMetadata>().Select(x =>
                new VariableFieldInfo(x, GetTypeName(x.Type), context.Options.ResolveFieldName(context, x),
                x.Name.ToCamelCase(), x.Type is ComplexTypeMetadata or ArrayTypeMetadata, GetValueLiteralOrDefault(x)))
                .ToArray();
    }


    public CSharpElement Build(string? path, bool isInternal)
    {
        var cls = new CSharpClass(_context.ClassName);
        if (isInternal)
        {
            cls.Visibility = CSharpVisibility.Internal;
        }

        cls.AddCommentsForClass(_context.Metadata);

        cls.Modifiers |= CSharpModifiers.Unsafe;

        cls.AddTypeSupportAttribute(_context.Metadata.ToString());
        cls.AddTypeSupportInterface(_context);

        cls.Members.Add(EmitTypeSupportNameProperty());
        cls.Members.Add(EmitGetTypeSupportHandle());

        cls.Members.Add(EmitFullConstructor());
        cls.Members.Add(EmitRefConstructor());

        foreach (var c in EmitConstants())
            cls.Members.Add(c);

        foreach (var c in EmitProperties())
            cls.Members.Add(c);

        cls.Members.Add(EmitWriteTo());
        cls.Members.Add(EmitWriteToRef());

        cls.Members.Add(EmitCreateFrom());
        cls.Members.Add(EmitUnsafeCreate());
        cls.Members.Add(EmitUnsafeDestroy());
        cls.Members.Add(EmitUnsafeInitialize());
        cls.Members.Add(EmitUnsafeFinalize());
        cls.Members.Add(EmitUnsafeInitializeSequence());
        cls.Members.Add(EmitUnsafeFinalizeSequence());

        cls.Members.Add(PrivStructBuilder.Build(_context));
        cls.Members.Add(PrivStructSequenceBuilder.Build(_context));

        foreach (var m in cls.Members.OfType<CSharpMethod>())
        {
            m.Attributes.Add(Attributes.DebuggerNonUserCode);
            m.Attributes.Add(Attributes.GeneratedCode);
        }

        if (path != null)
        {
            var ns = new CSharpNamespace(_context.Namespace);
            ns.Members.Add(cls);

            var file = new CSharpGeneratedFile(path);
            file.Members.Clear();
            file.Members.Add(new CSharpFreeMember { Text = "#nullable enable" });
            file.Members.Add(ns);
            return file;
        }

        return cls;
    }

    private CSharpFreeMember EmitFullConstructor()
    {
        var builder = new StringBuilder();

        builder.AppendLine($"""
            /// <summary>
            /// Create a new instance of <see cref="{_context.ClassName}"/> with fields initialized to specified values.
            /// </summary>
            """);

        foreach (var field in _variables)
        {
            builder.AppendFormat("/// <param name='{0}'>", field.ParameterName);
            builder.AppendLine();

            foreach (var c in field.Metadata.Comments)
            {
                builder.AppendLine($"/// {c}");
            }

            if (field.Metadata.Comments.Any())
            {
                builder.AppendFormat("/// <para>(originally defined as: <c><![CDATA[{0}]]></c>)</para>", field.Metadata);
                builder.AppendLine();
            }
            else
            {
                builder.AppendFormat("/// Originally defined as: <c><![CDATA[{0}]]></c>", field.Metadata);
                builder.AppendLine();
            }

            builder.AppendLine("/// </param>");
        }

        builder.AppendLine($"""
            [{Attributes.DebuggerNonUserCode}]
            [{Attributes.GeneratedCode}]
            """);

        if (_variables.Any())
        {
            builder.AppendLine($"public {_context.ClassName}(");

            for (var i = 0; i < _variables.Length; i++)
            {
                builder.Append("    ");
                builder.Append($"{_variables[i].TypeName}");

                // _context.GetNormalizedFieldName(_context.Variables[i])
                if (_variables[i].IsNullable) builder.Append("?");

                builder.AppendFormat(" @{0} = {1}", _variables[i].ParameterName,
                    _variables[i].IsNullable ? "null" : _variables[i].DefaultValueLiteral);

                if (i < _variables.Length - 1) builder.AppendLine(",");
                else builder.AppendLine();
            }

            builder.AppendLine(")");

            builder.AppendLine("{");
            var previousLineIsBlank = true;
            for (var i = 0; i < _variables.Length; i++)
            {
                if (!_variables[i].IsNullable)
                {
                    builder.AppendFormat("    {0} = @{1};", _variables[i].PropertyName, _variables[i].ParameterName);
                    builder.AppendLine();
                    previousLineIsBlank = false;
                }
                else
                {
                    if (_variables[i].Metadata.Type.TryGetArraySize(out var arraySize))
                    {
                        if (!previousLineIsBlank) builder.AppendLine();
                        var arrType = (ArrayTypeMetadata)_variables[i].Metadata.Type;

                        builder.AppendLine($"    if (@{_variables[i].ParameterName} != null)");
                        builder.AppendLine("    {");
                        builder.AppendLine($"        {_variables[i].PropertyName} = @{_variables[i].ParameterName};");
                        builder.AppendLine("    }");
                        builder.AppendLine("    else");
                        builder.AppendLine("    {");
                        builder.AppendLine($"        {_variables[i].PropertyName} = new {GetTypeName(arrType.ElementType)}[{arraySize}];");


                        if (arrType.ElementType is PrimitiveTypeMetadata prim && prim.ValueType is PrimitiveTypes.String or PrimitiveTypes.WString)
                        {
                            builder.AppendLine($"        for (int i = 0; i < {arraySize}; i++)");
                            builder.AppendLine("        {");
                            builder.AppendLine($"            {_variables[i].PropertyName}[i] = \"\";");
                            builder.AppendLine("        }");
                        }
                        else if (arrType.ElementType is ComplexTypeMetadata ct)
                        {
                            builder.AppendLine($"        for (int i = 0; i < {arraySize}; i++)");
                            builder.AppendLine("        {");
                            builder.AppendLine($"           {_variables[i].PropertyName}[i] = new {GetTypeName(arrType.ElementType)}();");
                            builder.AppendLine("        }");
                        }

                        builder.AppendLine("    }");
                        if (i < _variables.Length - 1)
                        {
                            builder.AppendLine();
                            previousLineIsBlank = true;
                        }
                    }
                    else
                    {
                        builder.AppendFormat("    {0} = @{1} ?? {2};",
                            _variables[i].PropertyName, _variables[i].ParameterName, _variables[i].DefaultValueLiteral);
                        builder.AppendLine();
                        previousLineIsBlank = false;
                    }

                }
            }
            builder.AppendLine("}");
        }
        else
        {
            builder.AppendLine($"public {_context.ClassName}() {{ }}");
        }

        return new CSharpFreeMember() { Text = builder.ToString() };
    }

    private CSharpFreeMember EmitRefConstructor()
    {
        var builder = new StringBuilder();

        builder.AppendLine($"""
            /// <summary>
            /// Create a new instance of <see cref="{_context.ClassName}"/>, and copy its data from the specified <see cref="{_context.PrivStructName}"/> structure.
            /// </summary>
            /// <param name="priv">The <see cref="{_context.PrivStructName}"/> structure to be copied from.</param>
            /// <param name="textEncoding">Text encoding of the strings in the <see cref="{_context.PrivStructName}"/> structure and its containing structures, if any.</param>
            [{Attributes.DebuggerNonUserCode}]
            [{Attributes.GeneratedCode}]
            public {_context.ClassName}(in Priv priv, global::System.Text.Encoding textEncoding)
            """);
        builder.AppendLine("{");

        var previousLineIsBlank = true;
        for (var i = 0; i < _variables.Length; i++)
        {
            ReadField(builder, _variables[i], ref previousLineIsBlank, i == _variables.Length - 1);
        }

        builder.AppendLine("}");
        return new CSharpFreeMember() { Text = builder.ToString() };

        void ReadField(StringBuilder writer, VariableFieldInfo field, ref bool previousLineIsBlank, bool isLast)
        {
            var fieldName = field.PropertyName;
            switch (field.Metadata.Type)
            {
                case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.String:
                    writer.AppendLine($"    this.{fieldName} = global::Rosidl.Runtime.Interop.StringMarshal.CreatePooledString(priv.{fieldName}.AsSpan(), textEncoding);");
                    previousLineIsBlank = false;
                    break;
                case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.WString:
                    writer.AppendLine($"    this.{fieldName} = global::Rosidl.Runtime.Interop.StringMarshal.CreatePooledString(priv.{fieldName}.AsSpan());");
                    previousLineIsBlank = false;
                    break;
                case PrimitiveTypeMetadata prim:
                    writer.AppendLine($"    this.{fieldName} = priv.{fieldName};");
                    previousLineIsBlank = false;
                    break;
                case ComplexTypeMetadata complex:
                    writer.AppendLine($"    this.{fieldName} = new {_context.GetMessageClassReferenceName(complex)}(in priv.{fieldName}, textEncoding);");
                    previousLineIsBlank = false;
                    break;
                case ArrayTypeMetadata array:
                    if (array.TryGetArraySize(out var arraySize))
                    {
                        switch (array.ElementType)
                        {
                            case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.String or PrimitiveTypes.WString:
                                if (!previousLineIsBlank) writer.AppendLine();
                                writer.AppendLine($"    this.{fieldName} = new string[{arraySize}];");
                                writer.AppendLine($"    for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.AppendLine("    {");
                                if(prim.ValueType is PrimitiveTypes.String)
                                {
                                    writer.AppendLine($"        this.{fieldName}[i] = global::Rosidl.Runtime.Interop.StringMarshal.CreatePooledString(priv.{fieldName}[i].AsSpan(), textEncoding);");
                                }
                                else
                                {
                                    writer.AppendLine($"        this.{fieldName}[i] = global::Rosidl.Runtime.Interop.StringMarshal.CreatePooledString(priv.{fieldName}[i].AsSpan());");
                                }
                                writer.AppendLine("    }");
                                if (!isLast)
                                {
                                    writer.AppendLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            case PrimitiveTypeMetadata prim:
                                // var type = _context.GetPrimitiveTypeName(prim);
                                // writer.AppendLine($"    fixed ({type}* __p = priv.{fieldName}) this.{fieldName} = new global::System.Span<{type}>(__p, {arraySize}).ToArray();");
                                writer.AppendLine($"    this.{fieldName} = priv.{fieldName}.ToArray();");
                                previousLineIsBlank = false;
                                break;
                            case ComplexTypeMetadata complex:
                                if (!previousLineIsBlank) writer.AppendLine();
                                writer.AppendLine($"    this.{fieldName} = new {_context.GetMessageClassReferenceName(complex)}[{arraySize}];");
                                writer.AppendLine($"    for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.AppendLine("    {");
                                writer.AppendLine($"        this.{fieldName}[i] = new {_context.GetMessageClassReferenceName(complex)}(in priv.{fieldName}[i], textEncoding);");
                                writer.AppendLine("    }");
                                if (!isLast)
                                {
                                    writer.AppendLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            default:
                                throw new NotSupportedException();
                        }
                    }
                    else
                        switch (array.ElementType)
                        {
                            case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.String or PrimitiveTypes.WString:
                                if (!previousLineIsBlank) writer.AppendLine();
                                writer.AppendLine($"    this.{fieldName} = new string[priv.{fieldName}.Size];");
                                writer.AppendLine($"    var {fieldName}_span = priv.{fieldName}.AsSpan();");
                                writer.AppendLine($"    for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.AppendLine("    {");
                                if (prim.ValueType is PrimitiveTypes.String)
                                {
                                    writer.AppendLine($"        this.{fieldName}[i] = global::Rosidl.Runtime.Interop.StringMarshal.CreatePooledString({fieldName}_span[i].AsSpan(), textEncoding);");
                                }
                                else
                                {
                                    writer.AppendLine($"        this.{fieldName}[i] = global::Rosidl.Runtime.Interop.StringMarshal.CreatePooledString({fieldName}_span[i].AsSpan());");
                                }
                                writer.AppendLine("    }");
                                if (!isLast)
                                {
                                    writer.AppendLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            case PrimitiveTypeMetadata prim:
                                writer.AppendLine($"    this.{fieldName} = priv.{fieldName}.AsSpan().ToArray();");
                                previousLineIsBlank = false;
                                break;
                            case ComplexTypeMetadata complex:
                                if (!previousLineIsBlank) writer.AppendLine();
                                writer.AppendLine($"    this.{fieldName} = new {_context.GetMessageClassReferenceName(complex)}[priv.{fieldName}.Size];");
                                writer.AppendLine($"    var {fieldName}_span = priv.{fieldName}.AsSpan();");
                                writer.AppendLine($"    for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.AppendLine("    {");
                                writer.AppendLine($"        this.{fieldName}[i] = new {_context.GetMessageClassReferenceName(complex)}(in {fieldName}_span[i], textEncoding);");
                                writer.AppendLine("    }");
                                if (!isLast)
                                {
                                    writer.AppendLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            default:
                                throw new NotSupportedException();
                        }
                    break;
                default:
                    throw new NotSupportedException();
            }
        }
    }

    private CSharpField[] EmitConstants()
    {
        return _context.Metadata.Fields
            .OfType<ConstantFieldMetadata>()
            .Select(f =>
            {
                return new CSharpField(_context.GetNormalizedFieldName(f))
                {
                    Modifiers = CSharpModifiers.Const,
                    Visibility = CSharpVisibility.Public,
                    InitValue = GetValueLiteral(f.Type, f.Value),
                    FieldType = new CSharpFreeType(
                        _context.GetPrimitiveTypeName((PrimitiveTypeMetadata)f.Type))
                }
                .AddComments(f);
            })
            .ToArray();
    }

    private ICollection<CSharpElement> EmitProperties()
    {
        var items = new List<CSharpElement>();
        foreach (var f in _context.Metadata.Fields.OfType<VariableFieldMetadata>())
        {
            WriteVariableField(_context.ClassName, f, items);
        }
        return items;

        void WriteVariableField(string className, VariableFieldMetadata metadata, List<CSharpElement> output)
        {
            var type = GetTypeName(metadata.Type);
            var propName = _context.GetNormalizedFieldName(metadata);

            bool isFixed = false;
            int size = -1;
            string? backingFieldName = null;

            if (metadata.Type is PrimitiveTypeMetadata p && p.ValueType is PrimitiveTypes.String or PrimitiveTypes.WString && p.MaxLength != null)
            {
                size = p.MaxLength.Value;
            }
            else if (metadata.Type is ArrayTypeMetadata a && a.Length != null)
            {
                isFixed = !a.IsUpperBounded;
                size = a.Length.Value;
            }

            if (size >= 0)
            {
                backingFieldName = $"__backingField__{propName}";
                output.Add(new CSharpFreeMember() { Text = string.Format("private {0} {1};", type, backingFieldName) });
            }

            var prop = new CSharpProperty(propName)
            {
                ReturnType = new CSharpFreeType(type),
                Visibility = CSharpVisibility.Public
            };

            prop.AddComments(metadata);
            prop.Attributes.Add(Attributes.DebuggerNonUserCode);
            prop.Attributes.Add(Attributes.GeneratedCode);

            if (backingFieldName != null)
            {
                prop.GetBody = (w, e) => w.WriteLine("return " + backingFieldName + ";");

                if (metadata.Type is ArrayTypeMetadata or ComplexTypeMetadata ||
                        metadata.Type is PrimitiveTypeMetadata pm && pm.ValueType is PrimitiveTypes.String or PrimitiveTypes.WString)
                {
                    prop.AttributesForSet.Add(new CSharpFreeAttribute($"global::System.Diagnostics.CodeAnalysis.MemberNotNullAttribute(nameof({backingFieldName}))"));
                }

                if (isFixed)
                {
                    prop.SetBody = (w, e) =>
                    {
                        w.WriteLine(string.Format("""
                            {0} = value.Length == {1} ? value : throw new global::System.ArgumentException("Size of the array '{2}' must be {1}.");
                            """, backingFieldName, size, propName));
                    };
                }
                else
                {
                    prop.SetBody = (w, e) =>
                    {
                        w.WriteLine(string.Format("""
                            {0} = value.Length <= {1} ? value : throw new global::System.ArgumentException("Size of the array or string '{2}' must be less or equal than {1}.");
                            """, backingFieldName, size, propName));
                    };
                }
            }

            output.Add(prop);
        }
    }

    private CSharpMethod EmitCreateFrom()
    {
        var method = new CSharpMethod()
        {
            Name = "CreateFrom",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType("global::Rosidl.Runtime.IMessage")
        };

        method.Parameters.Add(new("data") { ParameterType = new CSharpFreeType("nint") });
        method.Parameters.Add(new CSharpParameter("textEncoding") { ParameterType = new CSharpFreeType("global::System.Text.Encoding") });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($"return new {_context.ClassName}(in global::System.Runtime.CompilerServices.Unsafe.AsRef<{_context.PrivStructName}>(data.ToPointer()), textEncoding);");
        };
        return method;
    }

    private CSharpMethod EmitUnsafeCreate()
    {
        var method = new CSharpMethod()
        {
            Name = "UnsafeCreate",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType("nint")
        };

        method.Body = (writer, element) =>
        {
            writer.WriteLine($"return new({_context.PrivStructName}.Create());");
        };
        return method;
    }

    private CSharpMethod EmitUnsafeInitialize()
    {
        var method = new CSharpMethod()
        {
            Name = "UnsafeInitialize",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new("data") { ParameterType = new CSharpFreeType("nint") });

        method.Body = (writer, element) =>
        {
            
            writer.WriteLine($"return {_context.PrivStructName}" +
                $".TryInitialize(out System.Runtime.CompilerServices.Unsafe.AsRef<" +
                    $"{_context.PrivStructName}>(data.ToPointer()));");
        };
        return method;
    }

    private CSharpMethod EmitUnsafeFinalize()
    {
        var method = new CSharpMethod()
        {
            Name = "UnsafeFinalize",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new("data") { ParameterType = new CSharpFreeType("nint") });

        method.Body = (writer, element) =>
        {

            writer.WriteLine($"{_context.PrivStructName}" +
                $".Finalize(ref System.Runtime.CompilerServices.Unsafe.AsRef<" +
                    $"{_context.PrivStructName}>(data.ToPointer()));");
        };
        return method;
    }

    private CSharpMethod EmitUnsafeInitializeSequence()
    {
        var method = new CSharpMethod()
        {
            Name = "UnsafeInitializeSequence",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Bool)
        };

        method.Parameters.Add(new("size") { ParameterType = new CSharpFreeType("int") });
        method.Parameters.Add(new("data") { ParameterType = new CSharpFreeType("nint") });

        method.Body = (writer, element) =>
        {

            writer.WriteLine($"return {_context.PrivStructSequenceName}" +
                $".TryInitialize(size, out System.Runtime.CompilerServices.Unsafe.AsRef<" +
                    $"{_context.PrivStructSequenceName}>(data.ToPointer()));");
        };
        return method;
    }

    private CSharpMethod EmitUnsafeFinalizeSequence()
    {
        var method = new CSharpMethod()
        {
            Name = "UnsafeFinalizeSequence",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new("data") { ParameterType = new CSharpFreeType("nint") });

        method.Body = (writer, element) =>
        {

            writer.WriteLine($"{_context.PrivStructSequenceName}" +
                $".Finalize(ref System.Runtime.CompilerServices.Unsafe.AsRef<" +
                    $"{_context.PrivStructSequenceName}>(data.ToPointer()));");
        };
        return method;
    }

    private CSharpMethod EmitUnsafeDestroy()
    {
        var method = new CSharpMethod()
        {
            Name = "UnsafeDestroy",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new("data") { ParameterType = new CSharpFreeType("nint") });

        method.Body = (writer, element) =>
        {
            writer.WriteLine($"{_context.PrivStructName}.Destroy(({_context.PrivStructName}*)data);");
        };
        return method;
    }

    private CSharpMethod EmitGetTypeSupportHandle()
    {
        var method = new CSharpMethod()
        {
            Name = "GetTypeSupportHandle",
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpFreeType("global::Rosidl.Runtime.TypeSupportHandle")
        };

        method.Body = (writer, element) =>
        {
            writer.WriteLine($$"""
                return new global::Rosidl.Runtime.TypeSupportHandle(_PInvoke(), global::Rosidl.Runtime.HandleType.Message);

                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("{{_context.TypeSupportLibraryName}}", EntryPoint = "rosidl_typesupport_c__get_message_type_support_handle__{{_context.Metadata.Package}}__{{_context.Metadata.SubFolder}}__{{_context.Metadata.Name}}")]
                static extern nint _PInvoke();
                """);
        };
        return method;
    }

    private CSharpProperty EmitTypeSupportNameProperty()
    {
        var prop = new CSharpProperty("TypeSupportName")
        {
            Modifiers = CSharpModifiers.Static,
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.String)
        };

        prop.Attributes.Add(Attributes.DebuggerNonUserCode);
        prop.Attributes.Add(Attributes.GeneratedCode);

        prop.GetBodyInlined = $"\"{_context.Metadata}\"";
        return prop;
    }

    private CSharpMethod EmitWriteTo()
    {
        var method = new CSharpMethod()
        {
            Name = "WriteTo",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("data") { ParameterType = new CSharpFreeType("nint") });
        method.Parameters.Add(new CSharpParameter("textEncoding") { ParameterType = new CSharpFreeType("global::System.Text.Encoding") });
        method.Body = (writer, element) =>
        {
            writer.WriteLine("WriteTo(ref global::System.Runtime.CompilerServices.Unsafe.AsRef<Priv>(data.ToPointer()), textEncoding);");
        };

        return method;
    }

    private CSharpMethod EmitWriteToRef()
    {
        var method = new CSharpMethod()
        {
            Name = "WriteTo",
            Visibility = CSharpVisibility.Public,
            ReturnType = new CSharpPrimitiveType(CSharpPrimitiveKind.Void)
        };

        method.Parameters.Add(new CSharpParameter("priv") { ParameterType = new CSharpFreeType(_context.PrivStructName).Ref(CSharpRefKind.Ref) });
        method.Parameters.Add(new CSharpParameter("textEncoding") { ParameterType = new CSharpFreeType("global::System.Text.Encoding") });

        method.Body = (writer, element) =>
        {
            var previousLineIsBlank = true;
            for (var i = 0; i < _context.Variables.Length; i++)
            {
                WriteField(writer, _context.Variables[i], ref previousLineIsBlank, i == _context.Variables.Length - 1);
            }
        };

        return method;

        void WriteField(CodeWriter writer, VariableFieldMetadata field, ref bool previousLineIsBlank, bool isLast)
        {
            var fieldName = _context.GetNormalizedFieldName(field);
            switch (field.Type)
            {
                case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.String:
                    writer.WriteLine($"priv.{fieldName}.CopyFrom(this.{fieldName}, textEncoding);");
                    previousLineIsBlank = false;
                    break;
                case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.WString:
                    writer.WriteLine($"priv.{fieldName}.CopyFrom(this.{fieldName});");
                    previousLineIsBlank = false;
                    break;
                case PrimitiveTypeMetadata prim:
                    writer.WriteLine($"priv.{fieldName} = this.{fieldName};");
                    previousLineIsBlank = false;
                    break;
                case ComplexTypeMetadata complex:
                    writer.WriteLine($"this.{fieldName}.WriteTo(ref priv.{fieldName}, textEncoding);");
                    previousLineIsBlank = false;
                    break;
                case ArrayTypeMetadata array:
                    if (array.TryGetArraySize(out var arraySize))
                    {
                        switch (array.ElementType)
                        {
                            case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.String or PrimitiveTypes.WString:
                                if (!previousLineIsBlank) writer.WriteLine();
                                writer.WriteLine($"for (int i = 0; i < {arraySize}; i++)");
                                writer.WriteLine("{");
                                if(prim.ValueType is PrimitiveTypes.String)
                                {
                                    writer.WriteLine($"    priv.{fieldName}[i].CopyFrom(this.{fieldName}[i], textEncoding);");
                                }
                                else
                                {
                                    writer.WriteLine($"    priv.{fieldName}[i].CopyFrom(this.{fieldName}[i]);");
                                }
                                writer.WriteLine("}");
                                if (!isLast)
                                {
                                    writer.WriteLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            case PrimitiveTypeMetadata prim:
                                var type = _context.GetPrimitiveTypeName(prim);
                                writer.WriteLine($"this.{fieldName}.CopyTo(priv.{fieldName});");
                                previousLineIsBlank = false;
                                break;
                            case ComplexTypeMetadata complex:
                                if (!previousLineIsBlank) writer.WriteLine();

                                var ct = _context.GetMessagePrivStructReferenceName(complex);
                                writer.WriteLine($"for (int i = 0; i < {arraySize}; i++)");
                                writer.WriteLine("{");
                                writer.WriteLine($"    this.{fieldName}[i].WriteTo(ref priv.{fieldName}[i], textEncoding);");
                                writer.WriteLine("}");
                                if (!isLast)
                                {
                                    writer.WriteLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            default:
                                throw new NotSupportedException();
                        }
                    }
                    else
                        switch (array.ElementType)
                        {
                            case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.String:
                                if (!previousLineIsBlank) writer.WriteLine();
                                writer.WriteLine($"priv.{fieldName} = new global::Rosidl.Runtime.Interop.CStringSequence(this.{fieldName}.Length);");
                                writer.WriteLine($"var {fieldName}_span = priv.{fieldName}.AsSpan();");
                                writer.WriteLine($"for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.WriteLine("{");
                                writer.WriteLine($"    {fieldName}_span[i].CopyFrom(this.{fieldName}[i], textEncoding);");
                                writer.WriteLine("}");
                                if (!isLast)
                                {
                                    writer.WriteLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            case PrimitiveTypeMetadata prim when prim.ValueType is PrimitiveTypes.WString:
                                if (!previousLineIsBlank) writer.WriteLine();
                                writer.WriteLine($"priv.{fieldName} = new global::Rosidl.Runtime.Interop.U16StringSequence(this.{fieldName}.Length);");
                                writer.WriteLine($"var {fieldName}_span = priv.{fieldName}.AsSpan();");
                                writer.WriteLine($"for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.WriteLine("{");
                                writer.WriteLine($"    {fieldName}_span[i].CopyFrom(this.{fieldName}[i]);");
                                writer.WriteLine("}");
                                if (!isLast)
                                {
                                    writer.WriteLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            case PrimitiveTypeMetadata prim:
                                writer.WriteLine($"priv.{fieldName}.CopyFrom(this.{fieldName});");
                                previousLineIsBlank = false;
                                break;
                            case ComplexTypeMetadata complex:
                                if (!previousLineIsBlank) writer.WriteLine();
                                writer.WriteLine($"priv.{fieldName} = new {_context.GetMessagePrivStructSequenceReferenceName(complex)}(this.{fieldName}.Length);");
                                writer.WriteLine($"var {fieldName}_span = priv.{fieldName}.AsSpan();");
                                writer.WriteLine($"for (int i = 0; i < this.{fieldName}.Length; i++)");
                                writer.WriteLine("{");
                                writer.WriteLine($"    this.{fieldName}[i].WriteTo(ref {fieldName}_span[i], textEncoding);");
                                writer.WriteLine("}");
                                if (!isLast)
                                {
                                    writer.WriteLine();
                                    previousLineIsBlank = true;
                                }
                                break;
                            default:
                                throw new NotSupportedException();
                        }
                    break;
                default:
                    throw new NotSupportedException();
            }
        }
    }

    private string GetTypeName(TypeMetadata metadata)
    {
        if (metadata is PrimitiveTypeMetadata p)
        {
            return _context.GetPrimitiveTypeName(p);
        }
        else if (metadata is ComplexTypeMetadata c)
        {
            return _context.GetMessageClassReferenceName(c);
        }
        else
        {
            var e = GetTypeName(((ArrayTypeMetadata)metadata).ElementType);
            return $"{e}[]";
        }
    }

    private string GetDefaultValueLiteral(TypeMetadata metadata)
    {
        if (metadata is PrimitiveTypeMetadata p)
        {
            return p.ValueType switch
            {
                PrimitiveTypes.String or PrimitiveTypes.WString => "\"\"",
                PrimitiveTypes.Bool => "false",
                PrimitiveTypes.Float64 => "0d",
                PrimitiveTypes.Float32 => "0f",
                _ => "0"
            };
        }
        else if (metadata is ComplexTypeMetadata c)
        {
            return $"new {_context.GetMessageClassReferenceName(c)}()";
        }
        else
        {
            var arr = (ArrayTypeMetadata)metadata;
            var elementName = GetTypeName(arr.ElementType);
            var arraySize = arr.Length.GetValueOrDefault();
            if (arraySize > 0 && !arr.IsUpperBounded)
            {
                var elements = string.Join(", ", Enumerable.Range(0, arraySize).Select(_ => GetDefaultValueLiteral(arr.ElementType)));
                return $"new {elementName}[{arr.Length}] {{ {elements} }}";
            }

            return $"global::System.Array.Empty<{elementName}>()";
        }
    }

    private string GetValueLiteral(TypeMetadata typeHint, object value)
    {
        if (typeHint is PrimitiveTypeMetadata p)
        {
            switch (value)
            {
                case double d: return $"{d}d";
                case float f: return $"{f}f";
                case string s:
                    s = s.Escape();
                    return $"\"{s}\"";
                case bool b:
                    return b ? "true" : "false";
                default:
                    return value.ToString()!;
            }
        }
        else if (typeHint is ArrayTypeMetadata arr)
        {
            var elementName = GetTypeName(arr.ElementType);
            var elements = string.Join(", ", ((Array)value).Cast<object>().Select(x => GetValueLiteral(arr.ElementType, x)));
            return $"new {elementName}[] {{ {elements} }}";
        }

        throw new NotSupportedException();
    }

    public string GetValueLiteralOrDefault(VariableFieldMetadata field)
        => field.DefaultValue is null ?
            GetDefaultValueLiteral(field.Type) : GetValueLiteral(field.Type, field.DefaultValue);

    private record VariableFieldInfo(VariableFieldMetadata Metadata, string TypeName,
        string PropertyName, string ParameterName, bool IsNullable, string DefaultValueLiteral);
}
