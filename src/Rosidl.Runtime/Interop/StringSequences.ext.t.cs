@output(CStringSequence.ext.g.cs, StructName=CStringSequence, NativeStructName=String)@
@output(U16StringSequence.ext.g.cs, StructName=U16StringSequence, NativeStructName=U16String)@

using System.Runtime.InteropServices;

namespace Rosidl.Runtime.Interop;

public unsafe partial struct @StructName@
{
    /// <summary>
    /// Create a <see cre="@StructName@"/> structure with a specific size.
    /// </summary>
    /// <param name="size">Size of the internal storage of the <see cre="@StructName@"/> structure to be allocated.</param>
    /// <remarks>
    /// The <see cre="@StructName@"/> initially has size and capacity equal to the <paramref name="size"/> parameter.
    /// The <see cre="@StructName@"/> should be deallocated using <see cref="Destroy(@StructName@*)"/> when it is no longer needed.
    // </remakrs>
    /// <returns>
    /// A pointer to the created <see cre="@StructName@"/> structure if successful, otherwise <see langword="null"/>.
    /// </returns>
    public static @StructName@* Create(int size)
    {
        return _PInovke((uint)size);

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__create")]
        static extern @StructName@* _PInovke(nuint size);
    }

    /// <summary>
    /// Destroy a <see cre="@StructName@"/> allocated with <see cref="Create(int)"/>.
    /// </summary>
    /// <remarks>Calling the function with an already deallocated sequence is a no-op.</remarks>
    public static void Destroy(@StructName@* sequence)
    {
        _PInovke(sequence);

        [DllImport("rosidl_runtime_c", EntryPoint = "rosidl_runtime_c__@NativeStructName@__Sequence__destroy")]
        static extern void _PInovke(@StructName@* sequence);
    }
}