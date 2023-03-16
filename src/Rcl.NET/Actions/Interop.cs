using System.Runtime.InteropServices;

namespace Rcl.Actions;

unsafe class DynamicFunctionTable
{
    public DynamicFunctionTable(string actionTypesupportName)
    {
        CopyFeedback = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool>)GetFunction(actionTypesupportName, "Feedback", "copy");

        CreateResult = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint>)GetFunction(actionTypesupportName, "Result", "create");
        DestroyResult = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void>)GetFunction(actionTypesupportName, "Result", "destroy");
        CopyResult = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool>)GetFunction(actionTypesupportName, "Result", "copy");

        CreateGoal = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint>)GetFunction(actionTypesupportName, "Goal", "create");
        DestroyGoal = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void>)GetFunction(actionTypesupportName, "Goal", "destroy");
        CopyGoal = (delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool>)GetFunction(actionTypesupportName, "Goal", "copy");
    }

    public RosMessageBuffer CreateResultBuffer()
    {
        return new RosMessageBuffer(CreateResult(), (p, self) => ((DynamicFunctionTable)self!).DestroyResult(p), this);
    }

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

    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, nint, bool> CopyGoal, CopyFeedback, CopyResult;

    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<nint> CreateResult, CreateGoal;

    public readonly delegate* unmanaged[Cdecl, SuppressGCTransition]<nint, void> DestroyResult, DestroyGoal;

}