using Rcl.SafeHandles;

namespace Rcl.Internal;

unsafe class RclArgumentsImpl : IDisposable
{
    private static readonly RclAllocator _allocator = RclAllocator.Default;
    private readonly SafeArgumentsHandle _handle;

    public RclArgumentsImpl(string[] args)
    {
        _handle = new SafeArgumentsHandle(args);
    }

    public RclArgumentsImpl(IntPtr handle)
    {
        _handle = new SafeArgumentsHandle( handle);
    }

    public int[] GetUnparsedArgumentIndices()
    {
        var count = rcl_arguments_get_count_unparsed(_handle.Object);
        if (count == 0)
        {
            return Array.Empty<int>();
        }


        int* p = null;
        try
        {
            rcl_arguments_get_unparsed(_handle.Object, _allocator.Object, &p);
            return new Span<int>(p, count).ToArray();
        }
        finally
        {
            if (p != null) _allocator.Deallocate(p);
        }
    }

    public int[] GetUnparsedRosArgumentIndices()
    {
        var count = rcl_arguments_get_count_unparsed_ros(_handle.Object);
        if (count == 0)
        {
            return Array.Empty<int>();
        }

        int* p = null;
        try
        {
            rcl_arguments_get_unparsed_ros(_handle.Object, _allocator.Object, &p);
            return new Span<int>(p, count).ToArray();
        }
        finally
        {
            if (p != null) _allocator.Deallocate(p);
        }
    }

    public string[] GetParamFiles()
    {
        var count = rcl_arguments_get_param_files_count(_handle.Object);
        if (count == 0)
        {
            return Array.Empty<string>();
        }

        var items = new string[count];

        sbyte** p = null;
        try
        {
            rcl_arguments_get_param_files(_handle.Object, _allocator.Object, &p);
            for (var i = 0; i < count; i++)
            {
                items[i] = new(p[i]);
            }
        }
        finally
        {
            if (p != null) _allocator.Deallocate(p);
        }

        return items;
    }

    public NodeParameter[] GetParameters()
    {
        rcl_params_t* p;
        rcl_arguments_get_param_overrides(_handle.Object, &p);

        var count = p->num_nodes;

        var nodeNames = new string[count];
        var nodeParams = new Dictionary<string, object>[count];

        for (var i = 0; i < nodeNames.Length; i++)
        {
            var pDict = new Dictionary<string, object>();
            nodeNames[i] = new(p->node_names[i]);
            nodeParams[i] = pDict;

            var pNodeParam = &p->@params[i];
            var sz = pNodeParam->num_params.Value.ToUInt32();
            for (var j = 0; j < sz; j++)
            {
                var k = new string(pNodeParam->parameter_names[j]);
                var v = VariantToObject(&pNodeParam->parameter_values[j]);
                pDict[k] = v;
            }
        }

        return nodeNames.Zip(nodeParams, (name, p) => new NodeParameter(name, p)).ToArray();
    }

    private object VariantToObject(rcl_variant_t* v)
    {
        if (v->bool_array_value != null)
        {
            return new Span<bool>(
                v->bool_array_value->values,
                (int)v->bool_array_value->size.Value).ToArray();
        }
        else if (v->bool_value != null)
        {
            return *v->bool_value;
        }
        else if (v->byte_array_value != null)
        {
            return new Span<byte>(
                v->byte_array_value->values,
                (int)v->byte_array_value->size.Value).ToArray();
        }
        else if (v->double_array_value != null)
        {
            return new Span<double>(
                v->double_array_value->values,
                (int)v->double_array_value->size.Value).ToArray();
        }
        else if (v->double_value != null)
        {
            return *v->double_value;
        }
        else if (v->integer_array_value != null)
        {
            return new Span<long>(
                v->integer_array_value->values,
                (int)v->integer_array_value->size.Value).ToArray();
        }
        else if (v->integer_value != null)
        {
            return *v->integer_value;
        }
        else if (v->string_array_value != null)
        {
            var sz = v->string_array_value->size;
            if (sz == 0) return Array.Empty<string>();

            var results = new string[sz];
            for (var i = 0; i < results.Length; i++)
            {
                results[i] = new(v->string_array_value->data[i]);
            }

            return results;
        }
        else
        {
            return new string(v->string_value);
        }
    }

    public void Dispose()
    {
        _handle.Dispose();
    }
}

record struct NodeParameter(string Node, Dictionary<string, object> Parameters);