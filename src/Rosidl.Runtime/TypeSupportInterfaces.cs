using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rosidl.Runtime;

/// <summary>
/// Provide access to type support of ROS interface definition.
/// </summary>
public interface ITypeSupport
{
    /// <summary>
    /// Gets the type support name of the interface.
    /// </summary>
    abstract static string TypeSupportName { get; }

    /// <summary>
    /// Gets a type support handle of the interface,
    /// which can be used for performing introspection on dynamically created messages.
    /// </summary>
    /// <returns>A <see cref="TypeSupportHandle"/> of type <see cref="HandleType.Message"/>,
    /// <see cref="HandleType.Service"/> or <see cref="HandleType.Action"/>, depending on
    /// the type of the ROS interface.
    /// </returns>
    abstract static TypeSupportHandle GetTypeSupportHandle();
}

/// <summary>
/// Represents an ROS service interface.
/// </summary>
public interface IService : ITypeSupport
{
}

/// <summary>
/// Represents an ROS service interface with request of type <typeparamref name="TRequest"/> and response of type <typeparamref name="TResponse"/>.
/// </summary>
public interface IService<TRequest, TResponse> : IService
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
}

/// <summary>
/// Represents an ROS action interface.
/// </summary>
public interface IAction : ITypeSupport
{
}

/// <summary>
/// Represents an ROS action interface with goal of type <typeparamref name="TGoal"/>, result of type <typeparamref name="TResult"/> and 
/// feedback of type <typeparamref name="TFeedback"/>.
/// </summary>
public interface IAction<TGoal, TResult, TFeedback> : IAction
    where TGoal : IActionGoal
    where TResult : IActionResult
    where TFeedback : IActionFeedback
{
}

/// <summary>
/// Represents an ROS message interface.
/// </summary>
public interface IMessage : ITypeSupport
{
    /// <summary>
    /// Create a managed message instance by copying data from native message buffer.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message structure.</param>
    /// <param name="textEncoding">Encoding of the string values in the message.</param>
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
    /// Initialize a previously allocated native message structure.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message structure.</param>
    /// <returns><see langword="true"/> is the initialization is successful, otherwise <see langword="false"/>.</returns>
    abstract static bool UnsafeInitialize(nint data);

    /// <summary>
    /// Finalize a native message structure, but without freeing the message pointer.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message structure.</param>
    abstract static void UnsafeFinalize(nint data);

    /// <summary>
    /// Initialize a previously allocated native message sequence structure.
    /// </summary>
    /// <param name="size">Size of the sequence.</param>
    /// <param name="data">A pointer to the underlying ROS message sequence structure.</param>
    /// <returns><see langword="true"/> is the initialization is successful, otherwise <see langword="false"/>.</returns>
    abstract static bool UnsafeInitializeSequence(int size, nint data);

    /// <summary>
    /// Finalize a native message sequence structure, but without freeing the message sequence pointer.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message sequence structure.</param>
    abstract static void UnsafeFinalizeSequence(nint data);

    /// <summary>
    /// Write data of current message instance into native message buffer.
    /// </summary>
    /// <param name="data">A pointer to the underlying ROS message structure.</param>
    /// <param name="textEncoding">Encoding of the string values in the message.</param>
    void WriteTo(nint data, Encoding textEncoding);
}

/// <summary>
/// Represents an ROS service request message.
/// </summary>
public interface IServiceRequest : IMessage { }

/// <summary>
/// Represents an ROS service response message.
/// </summary>
public interface IServiceResponse : IMessage { }

/// <summary>
/// Represents an ROS action goal message.
/// </summary>
public interface IActionGoal : IMessage { }

/// <summary>
/// Represents an ROS action result message.
/// </summary>
public interface IActionResult : IMessage { }

/// <summary>
/// Represents an ROS action feedback message.
/// </summary>
public interface IActionFeedback : IMessage { }