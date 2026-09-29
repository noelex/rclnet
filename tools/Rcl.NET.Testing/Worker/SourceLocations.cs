using System.Reflection.Metadata;
using System.Reflection.Metadata.Ecma335;
using System.Reflection.PortableExecutable;

namespace Rcl.NET.Testing;

internal static class SourceLocations
{
    internal static Dictionary<string, (string File, int Line)> Read(string assembly)
    {
        var result = new Dictionary<string, (string, int)>();
        var pdb = Path.ChangeExtension(assembly, ".pdb");
        if (!File.Exists(pdb))
        {
            return result;
        }

        try
        {
            using var peStream = File.OpenRead(assembly);
            using var pe = new PEReader(peStream);
            var metadata = pe.GetMetadataReader();
            using var pdbStream = File.OpenRead(pdb);
            using var provider = MetadataReaderProvider.FromPortablePdbStream(pdbStream);
            var symbols = provider.GetMetadataReader();
            foreach (var handle in symbols.MethodDebugInformation)
            {
                var info = symbols.GetMethodDebugInformation(handle);
                var point = info.GetSequencePoints().FirstOrDefault(p => !p.IsHidden);
                if (point.StartLine == 0 || point.Document.IsNil)
                {
                    continue;
                }

                var methodHandle = info.GetStateMachineKickoffMethod();
                if (methodHandle.IsNil)
                {
                    methodHandle = MetadataTokens.MethodDefinitionHandle(MetadataTokens.GetRowNumber(handle));
                }

                var method = metadata.GetMethodDefinition(methodHandle);
                var type = metadata.GetTypeDefinition(method.GetDeclaringType());
                var name = metadata.GetString(type.Namespace) + "." + metadata.GetString(type.Name) + "." + metadata.GetString(method.Name);
                result.TryAdd(name, (symbols.GetString(symbols.GetDocument(point.Document).Name), point.StartLine));
            }
        }
        catch (BadImageFormatException)
        {
            // Source navigation is optional when a build supplies non-portable symbols.
        }

        return result;
    }
}
