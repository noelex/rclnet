using System.Runtime.CompilerServices;

namespace Rosidl.Messages.UniqueIdentifier;

partial class UUID
{
    public unsafe partial struct Priv
    {
        public Priv(Guid guid)
        {
            CopyFrom(guid);
        }

        public void CopyFrom(Guid guid)
        {
            // UUID has the same fixed 16-byte layout in both ROSIDL ABIs.
            fixed (Priv* p = &this) Unsafe.Copy(p, ref guid);
        }

        public Guid ToGuid()
        {
            fixed (Priv* p = &this) return Unsafe.AsRef<Guid>(p);
        }

        public static implicit operator Guid(Priv uuid)
        {
            return uuid.ToGuid();
        }

        public static implicit operator Priv(Guid guid)
        {
            return new Priv(guid);
        }
    }

    public unsafe partial struct PrivV2
    {
        public PrivV2(Guid guid)
        {
            CopyFrom(guid);
        }

        public void CopyFrom(Guid guid)
        {
            // UUID has the same fixed 16-byte layout in both ROSIDL ABIs.
            fixed (PrivV2* p = &this) Unsafe.Copy(p, ref guid);
        }

        public Guid ToGuid()
        {
            fixed (PrivV2* p = &this) return Unsafe.AsRef<Guid>(p);
        }

        public static implicit operator Guid(PrivV2 uuid)
        {
            return uuid.ToGuid();
        }

        public static implicit operator PrivV2(Guid guid)
        {
            return new PrivV2(guid);
        }
    }
}
