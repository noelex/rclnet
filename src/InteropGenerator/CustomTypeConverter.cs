using CppAst.CodeGen.CSharp;
using CppAst;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteropGenerator;

public class CustomTypeConverter : ICSharpConverterPlugin
{
    private static readonly CppQualifiedType ConstChar = new CppQualifiedType(CppTypeQualifier.Const, CppPrimitiveType.Char);
    private static readonly CppQualifiedType ConstVoid = new CppQualifiedType(CppTypeQualifier.Const, CppPrimitiveType.Void);

    /// <inheritdoc />
    public void Register(CSharpConverter converter, CSharpConverterPipeline pipeline)
    {
        pipeline.GetCSharpTypeResolvers.Add(GetCSharpType);
    }

    public static CSharpType GetCSharpType(CSharpConverter converter, CppType cppType, CSharpElement context, bool nested)
    {
        // Early exit for primitive types
        if (cppType is CppPrimitiveType cppPrimitiveType)
        {
            // Special case for bool
            return cppPrimitiveType.Kind == CppPrimitiveKind.Bool
                ? GetBoolType(converter)
                : CSharpHelper.GetCSharpPrimitive(cppPrimitiveType);
        }

        // Check if a particular CppType has been already converted
        var csType = converter.FindCSharpType(cppType);
        if (csType != null)
        {
            return csType;
        }

        // Pre-decode the type by extracting any const/pointer and get the element type directly
        DecodeSimpleType(cppType, out var isConst, out var isPointer, out var elementType);

        if (isPointer)
        {
            if (isConst && elementType.Equals(CppPrimitiveType.Char))
            {
                // const char* => string (with marshal)
                csType = GetStringType(converter);
            }
            else
            {
                var pointedCSharpType = converter.GetCSharpType(elementType, context, true);

                //var isParam = context is CSharpParameter;
                //var isReturn = context is CSharpMethod;
                //if (!nested && (isParam || isReturn))
                //{
                    switch (elementType.TypeKind)
                    {
                        case CppTypeKind.Array:
                            break;
                        case CppTypeKind.Reference:
                            break;
                        case CppTypeKind.Qualified:
                            var qualifiedType = (CppQualifiedType)elementType;
                            //csType = new CSharpRefType(qualifiedType.Qualifier == CppTypeQualifier.Const ? (isParam ? CSharpRefKind.In : CSharpRefKind.RefReadOnly) : CSharpRefKind.Ref, converter.GetCSharpType(qualifiedType.ElementType, context, true));
                            csType = new CSharpPointerType(converter.GetCSharpType(qualifiedType.ElementType, context, true));
                            break;
                        case CppTypeKind.Function:
                            csType = pointedCSharpType;
                            break;
                        case CppTypeKind.Typedef:
                            //csType = new CSharpRefType(CSharpRefKind.Ref, pointedCSharpType);
                            csType = new CSharpPointerType(pointedCSharpType);
                            break;
                        case CppTypeKind.StructOrClass:
                            // Is the struct is an opaque definition (which can is transformed into passing the struct directly as
                            // the struct contains the pointer)
                            if (pointedCSharpType is CSharpStruct csStruct && csStruct.IsOpaque)
                            {
                                csType = csStruct;
                            }
                            else
                            {
                                // csType = new CSharpRefType(isConst ? (isParam ? CSharpRefKind.In : CSharpRefKind.RefReadOnly) : CSharpRefKind.Ref, pointedCSharpType);
                                csType = new CSharpPointerType(pointedCSharpType);
                            }
                            break;
                        case CppTypeKind.Enum:
                            //csType = new CSharpRefType(CSharpRefKind.Ref, pointedCSharpType);
                            csType = new CSharpPointerType(pointedCSharpType);
                            break;
                        case CppTypeKind.TemplateParameterType:
                            break;
                        case CppTypeKind.Unexposed:
                            break;
                        case CppTypeKind.Primitive:
                            var cppPrimitive = (CppPrimitiveType)elementType;
                            if (cppPrimitive.Kind != CppPrimitiveKind.Void)
                            {
                                //csType = new CSharpRefType(CSharpRefKind.Ref, pointedCSharpType);
                                csType = new CSharpPointerType(pointedCSharpType);
                            }
                            break;
                        case CppTypeKind.Pointer:
                            //csType = isParam ? new CSharpRefType(CSharpRefKind.Out, pointedCSharpType) : null;
                            csType = new CSharpPointerType(pointedCSharpType);
                            break;
                    }
                //}
                //else
                //{
                //    switch (elementType.TypeKind)
                //    {
                //        case CppTypeKind.Array:
                //            break;
                //        case CppTypeKind.Reference:
                //            break;
                //        case CppTypeKind.Qualified:
                //            break;
                //        case CppTypeKind.Function:
                //            csType = converter.ConvertType(elementType, context);
                //            break;
                //        case CppTypeKind.Typedef:
                //            break;
                //        case CppTypeKind.StructOrClass:
                //            // Is the struct is an opaque definition (which can is transformed into passing the struct directly as
                //            // the struct contains the pointer)
                //            if (pointedCSharpType is CSharpStruct csStruct && csStruct.IsOpaque)
                //            {
                //                csType = new CSharpPointerType(csStruct);
                //            }
                //            else
                //            {
                //                // csType = new CSharpRefType(isConst ? (isParam ? CSharpRefKind.In : CSharpRefKind.RefReadOnly) : CSharpRefKind.Ref, pointedCSharpType);
                //                csType = new CSharpPointerType(pointedCSharpType);
                //            }
                //            break;
                //        case CppTypeKind.Enum:
                //            break;
                //        case CppTypeKind.TemplateParameterType:
                //            break;
                //        case CppTypeKind.Unexposed:
                //            break;
                //        case CppTypeKind.Primitive:
                //            var cppPrimitive = (CppPrimitiveType)elementType;
                //            if (cppPrimitive.Kind != CppPrimitiveKind.Void)
                //            {
                //                //csType = new CSharpRefType(CSharpRefKind.Ref, pointedCSharpType);
                //                csType = new CSharpPointerType(pointedCSharpType);
                //            }
                //            break;
                //        case CppTypeKind.Pointer:
                //            break;
                //    }
                //}

                // Any pointers that is not decoded to a simpler form is exposed as an IntPtr
                csType = csType ?? new CSharpPointerType(CSharpPrimitiveType.Void());
            }
        }
        else
        {
            switch (cppType.TypeKind)
            {
                case CppTypeKind.Array:

                    var arrayType = (CppArrayType)cppType;
                    var arrayElementType = arrayType.ElementType;

                    //if (arrayType.Size < 0 && arrayElementType.Equals(ConstChar))
                    //{
                    //    // const char[] => string (with marshal)
                    //    csType = GetStringType(converter);
                    //}
                    //else if (arrayType.Size > 0 && (arrayElementType.Equals(CppPrimitiveType.Char) || arrayElementType.Equals(CppPrimitiveType.WChar)))
                    //{
                    //    var fixedStrType = new CSharpTypeWithAttributes(CSharpPrimitiveType.String());
                    //    fixedStrType.Attributes.Add(new CSharpMarshalAttribute(CSharpUnmanagedKind.ByValTStr) { SizeConst = arrayType.Size });
                    //    csType = fixedStrType;
                    //}
                    //else
                    //{
                    if (arrayType.Size > 0/* && arrayElementType.GetCanonicalType() is CppPrimitiveType cppPrimitive && cppPrimitive.Kind != CppPrimitiveKind.Bool*/)
                    {
                        var csArrayElementType = converter.GetCSharpType(arrayElementType, context, true);
                        csType = new CSharpFixedArrayType(csArrayElementType, arrayType.Size);
                    }
                    else
                    {
                        var csArrayElementType = converter.GetCSharpType(arrayElementType, context, true);
                        csType = new CSharpPointerType(csArrayElementType);
                        //var typeWithAttributes = new CSharpTypeWithAttributes(csType);
                        //var attr = new CSharpMarshalAttribute(CSharpUnmanagedKind.LPArray);
                        //if (csArrayElementType is CSharpTypeWithAttributes csArrayElementTypeWithAttributes)
                        //{
                        //    var marshalAttributeForArrayElementType = GetMarshalAttributeOrNull(csArrayElementTypeWithAttributes.Attributes);
                        //    attr.ArraySubType = marshalAttributeForArrayElementType.UnmanagedType;
                        //}

                        //if (arrayType.Size >= 0)
                        //{
                        //    attr.SizeConst = arrayType.Size;
                        //}
                        //typeWithAttributes.Attributes.Add(attr);
                        //csType = typeWithAttributes;
                    }
                    //}
                    break;

                case CppTypeKind.Reference:
                    csType = new CSharpRefType(CSharpRefKind.Ref, converter.GetCSharpType(((CppReferenceType)cppType).ElementType, context, true));
                    break;
                case CppTypeKind.Qualified:
                    var qualifiedType = (CppQualifiedType)cppType;
                    csType = converter.GetCSharpType(qualifiedType.ElementType, context, true);
                    // TODO: Handle in parameters
                    break;
                case CppTypeKind.Function:
                    csType = converter.ConvertType(cppType, context);
                    break;
                case CppTypeKind.Typedef:
                    csType = converter.ConvertType(cppType, context);
                    break;
                case CppTypeKind.StructOrClass:
                    csType = converter.ConvertType(cppType, context);
                    break;
                case CppTypeKind.Enum:
                    break;
                case CppTypeKind.TemplateParameterType:
                    break;
                case CppTypeKind.Unexposed:
                    break;
            }
        }

        return csType!;
    }

    private static CSharpMarshalAttribute GetMarshalAttributeOrNull(List<CSharpAttribute> attributes)
    {
        foreach (var cSharpAttribute in attributes)
        {
            if (cSharpAttribute is CSharpMarshalAttribute csMarshalAttribute)
            {
                return csMarshalAttribute;
            }
        }

        return null!;
    }

    public static CSharpType GetBoolType(CSharpConverter converter)
    {
        //CSharpType boolType = CSharpPrimitiveType.Bool();
        //if (converter.Options.DefaultMarshalForBool != null)
        //{
        //    var boolTypeWithMarshal = new CSharpTypeWithAttributes(boolType);
        //    boolTypeWithMarshal.Attributes.Add(converter.Options.DefaultMarshalForBool.Clone());
        //    boolType = boolTypeWithMarshal;
        //}
        return CSharpPrimitiveType.Bool();
    }

    public static CSharpType GetStringType(CSharpConverter converter)
    {
        return new CSharpPointerType(CSharpPrimitiveType.Byte());

        //CSharpType strType = CSharpPrimitiveType.String();
        //if (converter.Options.DefaultMarshalForString != null)
        //{
        //    var boolTypeWithMarshal = new CSharpTypeWithAttributes(strType);
        //    boolTypeWithMarshal.Attributes.Add(converter.Options.DefaultMarshalForString.Clone());
        //    strType = boolTypeWithMarshal;
        //}

        //return strType;
    }

    /// <summary>
    /// Decode either `const XX*` or `XX*` to return XX, isPointer:true/false, isConst: true/false, with element type `XX`
    /// </summary>
    /// <param name="type"></param>
    /// <param name="isConst"></param>
    /// <param name="isPointer"></param>
    /// <param name="elementType"></param>
    private static void DecodeSimpleType(CppType type, out bool isConst, out bool isPointer, out CppType elementType)
    {
        isConst = false;
        isPointer = false;
        elementType = type;

        if (elementType is CppPointerType pointerType)
        {
            isPointer = true;
            elementType = pointerType.ElementType;
        }

        if (elementType is CppQualifiedType qualifiedType)
        {
            isConst = qualifiedType.Qualifier == CppTypeQualifier.Const;
            elementType = qualifiedType.ElementType;
        }
    }
}