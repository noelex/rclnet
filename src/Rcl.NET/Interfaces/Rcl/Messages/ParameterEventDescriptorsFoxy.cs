//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by ros2cs.
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;

#nullable enable

namespace Rosidl.Messages.Rcl
{
    /// <summary>
    /// This message contains descriptors of a parameter event.
    /// It was an atomic update.
    /// A specific parameter name can only be in one of the three sets.
    /// </summary>
    /// <remarks>
    /// Message interface definition for <c>rcl_interfaces/msg/ParameterEventDescriptors</c>.
    /// </remarks>
    [global::Rosidl.Runtime.TypeSupportAttribute("rcl_interfaces/msg/ParameterEventDescriptors")]
    internal unsafe partial class ParameterEventDescriptorsFoxy : global::Rosidl.Runtime.IMessage
    {
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public static string TypeSupportName => "rcl_interfaces/msg/ParameterEventDescriptors";
        
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public static global::Rosidl.Runtime.TypeSupportHandle GetTypeSupportHandle()
        {
            return new global::Rosidl.Runtime.TypeSupportHandle(_PInvoke(), global::Rosidl.Runtime.HandleType.Message);
            
            [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
            [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_typesupport_c", EntryPoint = "rosidl_typesupport_c__get_message_type_support_handle__rcl_interfaces__msg__ParameterEventDescriptors")]
            static extern nint _PInvoke();
        }
        
        /// <summary>
        /// Create a new instance of <see cref="ParameterEventDescriptors"/> with fields initialized to specified values.
        /// </summary>
        /// <param name='newParameters'>
        /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] new_parameters]]></c>
        /// </param>
        /// <param name='changedParameters'>
        /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] changed_parameters]]></c>
        /// </param>
        /// <param name='deletedParameters'>
        /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] deleted_parameters]]></c>
        /// </param>
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public ParameterEventDescriptorsFoxy(
            global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[]? @newParameters = null,
            global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[]? @changedParameters = null,
            global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[]? @deletedParameters = null
        )
        {
            NewParameters = @newParameters ?? global::System.Array.Empty<global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy>();
            ChangedParameters = @changedParameters ?? global::System.Array.Empty<global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy>();
            DeletedParameters = @deletedParameters ?? global::System.Array.Empty<global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy>();
        }
        
        
        /// <summary>
        /// Create a new instance of <see cref="ParameterEventDescriptors"/>, and copy its data from the specified <see cref="Priv"/> structure.
        /// </summary>
        /// <param name="priv">The <see cref="Priv"/> structure to be copied from.</param>
        /// <param name="textEncoding">Text encoding of the strings in the <see cref="Priv"/> structure and its containing structures, if any.</param>
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public ParameterEventDescriptorsFoxy(in Priv priv, global::System.Text.Encoding textEncoding)
        {
            this.NewParameters = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[priv.NewParameters.Size];
            var NewParameters_span = priv.NewParameters.AsSpan();
            for (int i = 0; i < this.NewParameters.Length; i++)
            {
                this.NewParameters[i] = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy(in NewParameters_span[i], textEncoding);
            }
        
            this.ChangedParameters = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[priv.ChangedParameters.Size];
            var ChangedParameters_span = priv.ChangedParameters.AsSpan();
            for (int i = 0; i < this.ChangedParameters.Length; i++)
            {
                this.ChangedParameters[i] = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy(in ChangedParameters_span[i], textEncoding);
            }
        
            this.DeletedParameters = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[priv.DeletedParameters.Size];
            var DeletedParameters_span = priv.DeletedParameters.AsSpan();
            for (int i = 0; i < this.DeletedParameters.Length; i++)
            {
                this.DeletedParameters[i] = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy(in DeletedParameters_span[i], textEncoding);
            }
        }
        
        
        /// <summary>
        /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] new_parameters]]></c>
        /// </summary>
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[] NewParameters { get; set; }
        
        /// <summary>
        /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] changed_parameters]]></c>
        /// </summary>
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[] ChangedParameters { get; set; }
        
        /// <summary>
        /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] deleted_parameters]]></c>
        /// </summary>
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy[] DeletedParameters { get; set; }
        
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public void WriteTo(nint data, global::System.Text.Encoding textEncoding)
        {
            WriteTo(ref global::System.Runtime.CompilerServices.Unsafe.AsRef<Priv>(data.ToPointer()), textEncoding);
        }
        
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public void WriteTo(ref Priv priv, global::System.Text.Encoding textEncoding)
        {
            priv.NewParameters = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy.PrivSequence(this.NewParameters.Length);
            var NewParameters_span = priv.NewParameters.AsSpan();
            for (int i = 0; i < this.NewParameters.Length; i++)
            {
                this.NewParameters[i].WriteTo(ref NewParameters_span[i], textEncoding);
            }
            
            priv.ChangedParameters = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy.PrivSequence(this.ChangedParameters.Length);
            var ChangedParameters_span = priv.ChangedParameters.AsSpan();
            for (int i = 0; i < this.ChangedParameters.Length; i++)
            {
                this.ChangedParameters[i].WriteTo(ref ChangedParameters_span[i], textEncoding);
            }
            
            priv.DeletedParameters = new global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy.PrivSequence(this.DeletedParameters.Length);
            var DeletedParameters_span = priv.DeletedParameters.AsSpan();
            for (int i = 0; i < this.DeletedParameters.Length; i++)
            {
                this.DeletedParameters[i].WriteTo(ref DeletedParameters_span[i], textEncoding);
            }
        }
        
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public static global::Rosidl.Runtime.IMessage CreateFrom(nint data, global::System.Text.Encoding textEncoding)
        {
            return new ParameterEventDescriptorsFoxy(in global::System.Runtime.CompilerServices.Unsafe.AsRef<Priv>(data.ToPointer()), textEncoding);
        }
        
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public static nint UnsafeCreate()
        {
            return new(global::Rosidl.Messages.Rcl.ParameterEventDescriptors.Priv.Create());
        }
        
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
        [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
        public static void UnsafeDestroy(nint data)
        {
            Priv.Destroy((Priv*)data);
        }
        
        /// <summary>
        /// This message contains descriptors of a parameter event.
        /// It was an atomic update.
        /// A specific parameter name can only be in one of the three sets.
        /// </summary>
        /// <remarks>
        /// Blittable native structure for <c>rcl_interfaces/msg/ParameterEventDescriptors</c>.
        /// </remarks>
        [global::System.Runtime.InteropServices.StructLayoutAttribute(global::System.Runtime.InteropServices.LayoutKind.Sequential)]
        public partial struct Priv : global::System.IEquatable<Priv>, global::System.IDisposable
        {
            /// <summary>
            /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] new_parameters]]></c>
            /// </summary>
            public global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy.PrivSequence NewParameters;
            
            /// <summary>
            /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] changed_parameters]]></c>
            /// </summary>
            public global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy.PrivSequence ChangedParameters;
            
            /// <summary>
            /// Originally defined as: <c><![CDATA[rcl_interfaces/ParameterDescriptor[] deleted_parameters]]></c>
            /// </summary>
            public global::Rosidl.Messages.Rcl.ParameterDescriptorFoxy.PrivSequence DeletedParameters;
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public Priv()
            {
                ThrowIfNonSuccess(TryInitialize(out this));
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public Priv(Priv src)
                : this(in src)
            {
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public Priv(in Priv src)
                : this()
            {
                CopyFrom(in src); 
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public Priv(Priv* src)
                : this()
            {
                CopyFrom(src); 
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void Dispose()
            {
                Finalize(ref this);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(Priv src)
            {
                ThrowIfNonSuccess(TryCopy(in src, out this));
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(in Priv src)
            {
                ThrowIfNonSuccess(TryCopy(in src, out this));
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(Priv* src)
            {
                fixed (Priv* pThis = &this)
                {
                    ThrowIfNonSuccess(TryCopy(src, pThis));
                }
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool TryCopy(in Priv input, out Priv output)
            {
                fixed (Priv* pInput = &input, pOutput = &output)
                {
                    return TryCopy(pInput, pOutput);
                }
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public bool Equals(Priv other)
            {
                return Priv.AreEqual(in other, in this);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public override bool Equals(object? obj) => obj is Priv s ? this.Equals(s) : false;
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public override int GetHashCode()
            {
                var __hashCode = new global::System.HashCode();
                __hashCode.Add(this.NewParameters);
                __hashCode.Add(this.ChangedParameters);
                __hashCode.Add(this.DeletedParameters);
                return __hashCode.ToHashCode();
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static bool operator ==(Priv lhs, Priv rhs)
            {
                return lhs.Equals(rhs);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static bool operator !=(Priv lhs, Priv rhs)
            {
                return !(lhs == rhs);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static Priv* Create()
            {
                return _PInvoke();
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__create")]
                static extern Priv* _PInvoke();
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static void Destroy(Priv* msg)
            {
                _PInvoke(msg);
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__destroy")]
                static extern void _PInvoke(Priv* msg);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool TryInitialize(out Priv msg)
            {
                fixed (Priv* pMsg = &msg)
                {
                    return _PInvoke(pMsg);
                }
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__init")]
                static extern bool _PInvoke(Priv* msg);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static void Finalize(ref Priv msg)
            {
                fixed (Priv* pMsg = &msg)
                {
                    _PInvoke(pMsg);
                }
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__fini")]
                static extern void _PInvoke(Priv* msg);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool AreEqual(in Priv lhs, in Priv rhs)
            {
                fixed (Priv* plhs = &lhs, prhs = &rhs)
                {
                    return _PInvoke(plhs, prhs);
                }
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__are_qual")]
                static extern bool _PInvoke(Priv* lhs, Priv* rhs);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool TryCopy(Priv* input, Priv* output)
            {
                return _PInvoke(input, output);
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__copy")]
                static extern bool _PInvoke(Priv* input, Priv* output);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static void ThrowIfNonSuccess(bool ret, [global::System.Runtime.CompilerServices.CallerMemberNameAttribute]
            string caller = "")
            {
                if (!ret)
                {
                    throw new global::Rosidl.Runtime.RosidlException($"An error occurred when calling 'global::Rosidl.Messages.Rcl.ParameterEventDescriptors.Priv.{caller}'.");
                }
            }
        }
        
        /// <summary>
        /// This message contains descriptors of a parameter event.
        /// It was an atomic update.
        /// A specific parameter name can only be in one of the three sets.
        /// </summary>
        /// <remarks>
        /// Blittable native sequence structure for <c>rcl_interfaces/msg/ParameterEventDescriptors</c>.
        /// </remarks>
        [global::System.Runtime.InteropServices.StructLayoutAttribute(global::System.Runtime.InteropServices.LayoutKind.Sequential)]
        public partial struct PrivSequence : global::System.IEquatable<PrivSequence>, global::System.IDisposable
        {
            private Priv* __data;
            
            private nuint __size;
            
            private nuint __capacity;
            
            public int Size => (int)__size;
            
            public int Capcacity => (int)__capacity;
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public PrivSequence()
                : this(0)
            {
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public PrivSequence(int size)
            {
                ThrowIfNonSuccess(TryInitialize(size, out this));
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public PrivSequence(PrivSequence src)
                : this(in src)
            {
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public PrivSequence(in PrivSequence src)
                : this()
            {
                CopyFrom(in src); 
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public PrivSequence(PrivSequence* src)
                : this()
            {
                CopyFrom(src); 
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public PrivSequence(System.ReadOnlySpan<Priv> src)
                : this(src.Length)
            {
                src.CopyTo(AsSpan());
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void Dispose()
            {
                Finalize(ref this);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public System.Span<Priv> AsSpan()
            {
                return new(__data, Size);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(System.ReadOnlySpan<Priv> src)
            {
                Finalize(ref this);
                ThrowIfNonSuccess(TryInitialize(src.Length, out this));
                src.CopyTo(AsSpan());
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(PrivSequence src)
            {
                ThrowIfNonSuccess(TryCopy(in src, out this));
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(in PrivSequence src)
            {
                ThrowIfNonSuccess(TryCopy(in src, out this));
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public void CopyFrom(PrivSequence* src)
            {
                fixed (PrivSequence* pThis = &this)
                {
                    ThrowIfNonSuccess(TryCopy(src, pThis));
                }
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool TryCopy(in PrivSequence input, out PrivSequence output)
            {
                fixed (PrivSequence* pInput = &input, pOutput = &output)
                {
                    return TryCopy(pInput, pOutput);
                }
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public bool Equals(PrivSequence other)
            {
                return PrivSequence.AreEqual(in other, in this);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public override bool Equals(object? obj) => obj is PrivSequence s ? this.Equals(s) : false;
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public override int GetHashCode()
            {
                return global::System.HashCode.Combine((nint)__data, __size, __capacity);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static bool operator ==(PrivSequence lhs, PrivSequence rhs)
            {
                return lhs.Equals(rhs);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static bool operator !=(PrivSequence lhs, PrivSequence rhs)
            {
                return !(lhs == rhs);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static PrivSequence* Create()
            {
                return _PInvoke();
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__Sequence__create")]
                static extern PrivSequence* _PInvoke();
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static void Destroy(PrivSequence* msg)
            {
                _PInvoke(msg);
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__Sequence__destroy")]
                static extern void _PInvoke(PrivSequence* msg);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool TryInitialize(int size, out PrivSequence msg)
            {
                fixed (PrivSequence* pMsg = &msg)
                {
                    return _PInvoke(pMsg, (uint)size);
                }
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__Sequence__init")]
                static extern bool _PInvoke(PrivSequence* msg, nuint size);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static void Finalize(ref PrivSequence msg)
            {
                fixed (PrivSequence* pMsg = &msg)
                {
                    _PInvoke(pMsg);
                }
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__Sequence__fini")]
                static extern void _PInvoke(PrivSequence* msg);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool AreEqual(in PrivSequence lhs, in PrivSequence rhs)
            {
                fixed (PrivSequence* plhs = &lhs, prhs = &rhs)
                {
                    return _PInvoke(plhs, prhs);
                }
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__Sequence__are_qual")]
                static extern bool _PInvoke(PrivSequence* lhs, PrivSequence* rhs);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            private static bool TryCopy(PrivSequence* input, PrivSequence* output)
            {
                return _PInvoke(input, output);
                
                [global::System.Runtime.InteropServices.SuppressGCTransitionAttribute]
                [global::System.Runtime.InteropServices.DllImportAttribute("rcl_interfaces__rosidl_generator_c", EntryPoint = "rcl_interfaces__msg__ParameterEventDescriptors__Sequence__copy")]
                static extern bool _PInvoke(PrivSequence* input, PrivSequence* output);
            }
            
            [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
            [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Rosidl.Runtime.Generator.CSharp", "1.0.0")]
            public static void ThrowIfNonSuccess(bool ret, [global::System.Runtime.CompilerServices.CallerMemberNameAttribute]
            string caller = "")
            {
                if (!ret)
                {
                    throw new global::Rosidl.Runtime.RosidlException($"An error occurred when calling 'global::Rosidl.Messages.Rcl.ParameterEventDescriptors.PrivSequence.{caller}'.");
                }
            }
        }
    }
}