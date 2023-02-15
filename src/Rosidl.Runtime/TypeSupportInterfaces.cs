using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime;

public interface ITypeSupport
{
    abstract static string TypeSupportName { get; }

    abstract static TypeSupportHandle GetTypeSupportHandle();
}

public interface IService : ITypeSupport
{
}

public interface IService<out TRequest, out TResponse> : IService
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
}

public interface IAction : ITypeSupport
{
}

public interface IAction<TGoal, TResult, TFeedback> : IAction
    where TGoal : IActionGoal
    where TResult : IActionResult
    where TFeedback : IActionFeedback
{
}

public interface IMessage : ITypeSupport
{
    /// <summary>
    /// Create a managed message instance by copying data from native message buffer.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message structure.</param>
    /// <returns>A managed message instance of the implementing type.</returns>
    abstract static IMessage CreateFrom(nint data, Encoding textEncoding);

    /// <summary>
    /// Allocate memory for the underlying ROS message structure.
    /// </summary>
    /// <remarks>
    /// The returned pointer must be free using <see cref="UnsafeDestroy(nint)"/>.
    /// </remarks>
    /// <returns>A pointer to the allocated memory.</returns>
    abstract static nint UnsafeCreate();

    /// <summary>
    /// Free memory allocated by <see cref="UnsafeCreate"/>.
    /// </summary>
    /// <param name="data"></param>
    abstract static void UnsafeDestroy(nint data);

    /// <summary>
    /// Write data of current message instance into native message buffer.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message structure.</param>
    void WriteTo(nint data, Encoding textEncoding);
}

public interface IServiceRequest : IMessage { }

public interface IServiceResponse : IMessage { }

public interface IActionGoal : IMessage { }

public interface IActionResult : IMessage { }

public interface IActionFeedback : IMessage { }