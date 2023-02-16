using System.Runtime.InteropServices;

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
            fixed (byte* p = Uuid)
                System.Runtime.CompilerServices.Unsafe.Copy(p, ref guid);
        }

        public Guid ToGuid()
        {
            fixed (Priv* p = &this)
                return System.Runtime.CompilerServices.Unsafe.AsRef<Guid>(p);
        }
    }
}
