using Rcl.Qos;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Rcl;

public interface IRclSubscription : IRclObject
{
    QosProfile ActualQos { get; }

    bool IsValid { get; }

    int Publishers { get; }

    string? Name { get; }
}

public interface IRclSubscription<T> : IRclSubscription, IObservable<T>
    where T : IMessage
{
    IAsyncEnumerable<T> ReadAllAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// An <see cref="IRclSubscription"/> that allows receiving messages using native message buffers.
/// </summary>
public interface IRclNativeSubscription : IRclSubscription
{
    IAsyncEnumerable<RosMessageBuffer> ReadAllAsync(CancellationToken cancellationToken = default);
}