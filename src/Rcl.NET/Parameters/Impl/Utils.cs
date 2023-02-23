using Rcl.Graph;
using Rcl.Interop;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace Rcl.Parameters.Impl;

internal static class Utils
{
    private static bool MatchNodeName(string node_name, string node_fqn)
    {
        // Update the regular expression ["/*" -> "(/\\w+)" and "/**" -> "(/\\w+)*"]
        var regex = node_name.Replace("/*", "(/\\w+)");
        return Regex.IsMatch(node_fqn, regex);
    }
    public unsafe static ParameterDictionary ResolveParameterOverrides(
           string nodeFqn,
           ParameterDictionary overrides,
           rcl_arguments_t* localArgs,
           rcl_arguments_t* globalArgs)
    {
        var result = new ParameterDictionary();
        LoadParameters(result, globalArgs);
        LoadParameters(result, localArgs);

        foreach (var (k, v) in overrides)
        {
            result[k] = v;
        }

        return result;

        void LoadParameters(ParameterDictionary result, rcl_arguments_t* source)
        {
            if (source == null)
            {
                return;
            }

            rcl_params_t* param;

            RclException.ThrowIfNonSuccess(
                rcl_arguments_get_param_overrides(source, &param));

            if (param == null)
            {
                return;
            }

            try
            {
                ExtractRclParams(nodeFqn, param, result);
            }
            finally
            {
                rcl_yaml_node_struct_fini(param);
            }
        }
    }

    private static unsafe void ExtractRclParams(string nodeFqn, rcl_params_t* src, ParameterDictionary dest)
    {
        for (int i = 0; i < (int)src->num_nodes.Value; i++)
        {
            var node_name = StringMarshal.CreatePooledString(src->node_names[i])!;
            if (!node_name.StartsWith('/'))
            {
                node_name = "/" + node_name;
            }

            if (!MatchNodeName(node_name, nodeFqn))
            {
                continue;
            }

            node_name = nodeFqn;

            rcl_node_params_t* c_params_node = &src->@params[i];

            for (int p = 0; p < (int)c_params_node->num_params.Value; ++p)
            {
                var c_param_name = StringMarshal.CreatePooledString(c_params_node->parameter_names[p]);
                if (c_param_name != null)
                {
                    var c_param_value = &c_params_node->parameter_values[p];
                    dest[c_param_name] = ConvertVariant(c_param_value);
                }
            }
        }

        static Variant ConvertVariant(rcl_variant_t* c_variant)
        {
            if (c_variant->bool_array_value != null)
            {
                var val = c_variant->bool_array_value;
                return new Span<bool>(val->values, (int)val->size.Value).ToArray();
            }
            else if (c_variant->bool_value != null)
            {
                return *c_variant->bool_value;
            }
            else if (c_variant->byte_array_value != null)
            {
                var val = c_variant->byte_array_value;
                return new Span<byte>(val->values, (int)val->size.Value).ToArray();
            }
            else if (c_variant->double_array_value != null)
            {
                var val = c_variant->double_array_value;
                return new Span<double>(val->values, (int)val->size.Value).ToArray();
            }
            else if (c_variant->double_value != null)
            {
                return *c_variant->double_value;
            }
            else if (c_variant->integer_array_value != null)
            {
                var val = c_variant->integer_array_value;
                return new Span<long>(val->values, (int)val->size.Value).ToArray();
            }
            else if (c_variant->integer_value != null)
            {
                return *c_variant->integer_value;
            }
            else if (c_variant->string_array_value != null)
            {
                var val = c_variant->string_array_value;
                var arr = new string[val->size.Value];
                for (int i = 0; i < arr.Length; i++)
                {
                    arr[i] = StringMarshal.CreatePooledString(val->data[i])!;
                }
                return arr;
            }
            else if (c_variant->string_value != null)
            {
                return StringMarshal.CreatePooledString(c_variant->string_value)!;
            }
            else
            {
                throw new RclException("Invalid c_variant: No parameter value set.");
            }
        }
    }
}
