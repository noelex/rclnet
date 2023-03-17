using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.Actions;

unsafe class MessageBufferHelper
{
    public MessageBufferHelper(string actionTypesupportName)
    {
        _createFeedback = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint>)GetFunction(actionTypesupportName, "Feedback", "create");
        _destroyFeedback = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void>)GetFunction(actionTypesupportName, "Feedback", "destroy");
        _copyFeedback = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool>)GetFunction(actionTypesupportName, "Feedback", "copy");

        _createResult = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint>)GetFunction(actionTypesupportName, "Result", "create");
        _destroyResult = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void>)GetFunction(actionTypesupportName, "Result", "destroy");
        _copyResult = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool>)GetFunction(actionTypesupportName, "Result", "copy");

        _createGoal = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint>)GetFunction(actionTypesupportName, "Goal", "create");
        _destroyGoal = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void>)GetFunction(actionTypesupportName, "Goal", "destroy");
        _copyGoal = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool>)GetFunction(actionTypesupportName, "Goal", "copy");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public RosMessageBuffer CreateResultBuffer()
        => new(_createResult(), (p, self) => ((MessageBufferHelper)self!)._destroyResult(p), this);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public RosMessageBuffer CreateFeedbackBuffer()
        => new(_createFeedback(), (p, self) => ((MessageBufferHelper)self!)._destroyFeedback(p), this);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public RosMessageBuffer CreateGoalBuffer()
        => new(_createGoal(), (p, self) => ((MessageBufferHelper)self!)._destroyGoal(p), this);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool CopyGoal(nint src, nint dest) => _copyGoal(src, dest);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool CopyFeedback(nint src, nint dest) => _copyFeedback(src, dest);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool CopyResult(nint src, nint dest) => _copyResult(src, dest);

    private static (string pkg, string subfolder, string name) BreakName(string pkg)
    {
        var parts = pkg.Split('/');
        return (parts[0], parts[1], parts[2]);
    }

    private static nint GetFunction(string actionTypesupportName, string subType, string funcName)
    {
        var (pkg, sub, name) = BreakName(actionTypesupportName);
        var symName = $"{pkg}__{sub}__{name}_{subType}__{funcName}";
        var libName = $"{pkg}__rosidl_generator_c";

        var lib = NativeLibrary.Load(libName, System.Reflection.Assembly.GetExecutingAssembly(), null);
        return NativeLibrary.GetExport(lib, symName);
    }

    private readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool> _copyGoal, _copyFeedback, _copyResult;

    private readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<nint> _createResult, _createGoal, _createFeedback;

    private readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void> _destroyResult, _destroyGoal, _destroyFeedback;

}