using CppAst.CodeGen.Common;
using CppAst.CodeGen.CSharp;

namespace Rosidl.Generator.CSharp.Helpers
{
    internal class XmlComment : CSharpComment
    {
        public XmlComment(params string[] comments)
        {
            Children.AddRange(comments);
        }

        public new List<string> Children { get; set; } = new();

        public override void DumpTo(CodeWriter writer)
        {
            foreach (var line in Children)
            {
                writer.Write("/// ");
                writer.WriteLine(line);
            }
        }
    }

    class XmlCommentGroup : CSharpComment
    {
        public override void DumpTo(CodeWriter writer)
        {
            foreach (var child in Children)
            {
                child.DumpTo(writer);
            }
        }
    }

    internal static class XmlCommentHelper
    {
        private static XmlComment EmitXmlComment(string tagName, string[]? contents = null, Dictionary<string, string>? attributes = null)
        {
            var attrString = attributes == null ? "" : " " + string.Join(" ", attributes.Select(x => $"{x.Key}=\"{x.Value}\""));

            var comment = new XmlComment();
            if (contents == null)
            {
                comment.Children.Add($"<{tagName}{attrString}/>");
            }
            else
            {
                comment.Children.Add($"<{tagName}{attrString}>");
                foreach (var item in contents)
                {
                    comment.Children.Add(item);
                }
                comment.Children.Add($"</{tagName}>");
            }

            return comment;
        }

        public static XmlComment EmitSummary(params string[] contents)
        {
            return EmitXmlComment("summary", contents);
        }

        public static XmlComment EmitParam(string name, params string[] contents)
        {
            return EmitXmlComment("param", contents, new() { ["name"] = name });
        }

        public static XmlComment EmitReturns(params string[] contents)
        {
            return EmitXmlComment("returns", contents);
        }

        public static XmlComment EmitRemarks(params string[] contents)
        {
            return EmitXmlComment("remarks", contents);
        }

        public static XmlComment EmitPara(params string[] contents)
        {
            return EmitXmlComment("para", contents);
        }

        public static XmlComment EmitCref(string cref)
        {
            return EmitXmlComment("see", attributes: new() { ["cref"] = cref });
        }

        public static XmlComment EmitLangword(string langword)
        {
            return EmitXmlComment("see", attributes: new() { ["langword"] = langword });
        }

        public static T AddComments<T>(this T element, FieldMetadata field)
            where T : ICSharpWithComment
        {
            element.Comment = new XmlCommentGroup();
            if (field.Comments.Length > 0)
            {
                element.Comment.Children.Add(EmitSummary(field.Comments));
                element.Comment.Children.Add(EmitRemarks($"Originally defined as: <c><![CDATA[{field}]]></c>"));
            }
            else
            {
                element.Comment.Children.Add(EmitSummary($"Originally defined as: <c><![CDATA[{field}]]></c>"));
            }

            return element;
        }

        public static T AddComments<T>(this T element, ServiceMetadata service)
            where T : ICSharpWithComment
        {
            element.Comment = new XmlCommentGroup();

            var description = $"Service interface definition for <c>{service}</c>.";
            if (service.Comments.Length > 0)
            {
                element.Comment.Children.Add(EmitSummary(service.Comments));
                element.Comment.Children.Add(EmitRemarks(description));
            }
            else
            {
                element.Comment.Children.Add(EmitSummary(description));
            }
            return element;
        }

        public static T AddCommentsForClass<T>(this T element, MessageMetadata message)
            where T : ICSharpWithComment
        {
            element.Comment = new XmlCommentGroup();

            var description = $"Message interface definition for <c>{message}</c>.";
            if (message.Comments.Length > 0)
            {
                element.Comment.Children.Add(EmitSummary(message.Comments));
                element.Comment.Children.Add(EmitRemarks(description));
            }
            else
            {
                element.Comment.Children.Add(EmitSummary(description));
            }
            return element;
        }

        public static T AddComments<T>(this T element, ActionMetadata message)
    where T : ICSharpWithComment
        {
            element.Comment = new XmlCommentGroup();

            var description = $"Action interface definition for <c>{message}</c>.";
            if (message.Comments.Length > 0)
            {
                element.Comment.Children.Add(EmitSummary(message.Comments));
                element.Comment.Children.Add(EmitRemarks(description));
            }
            else
            {
                element.Comment.Children.Add(EmitSummary(description));
            }
            return element;
        }

        public static T AddCommentsForStruct<T>(this T element, MessageMetadata message)
            where T : ICSharpWithComment
        {
            element.Comment = new XmlCommentGroup();

            var description = $"Blittable native structure for <c>{message}</c>.";
            if (message.Comments.Length > 0)
            {
                element.Comment.Children.Add(EmitSummary(message.Comments));
                element.Comment.Children.Add(EmitRemarks(description));
            }
            else
            {
                element.Comment.Children.Add(EmitSummary(description));
            }
            return element;
        }

        public static T AddCommentsForStructSequence<T>(this T element, MessageMetadata message)
            where T : ICSharpWithComment
        {
            element.Comment = new XmlCommentGroup();

            var description = $"Blittable native sequence structure for <c>{message}</c>.";
            if (message.Comments.Length > 0)
            {
                element.Comment.Children.Add(EmitSummary(message.Comments));
                element.Comment.Children.Add(EmitRemarks(description));
            }
            else
            {
                element.Comment.Children.Add(EmitSummary(description));
            }
            return element;
        }
    }
}
