using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Messages.UniqueIdentifier;

public partial class UUID
{
    public partial struct Priv
    {
        public Priv(Guid guid)
        {
            CopyFrom(guid);
        }

        public void  CopyFrom(Guid guid)
        {
            Uuid.CopyFrom(MemoryMarshal.AsBytes(new Span<Guid>(ref guid)));
        }

        public Guid ToGuid() => new (Uuid.AsSpan());
    }
}
