using System.Runtime.CompilerServices;

namespace Rosidl.Messages.UniqueIdentifier;

public partial class UUID
{
    public unsafe partial struct Priv
    {
        public Priv(Guid guid)
        {
            CopyFrom(guid);
        }

        public void CopyFrom(Guid guid)
        {
            fixed (byte* p = Uuid) Unsafe.Copy(p, ref guid);
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
}
