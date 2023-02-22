using Rcl.Graph;
using Rcl.Interop;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace Rcl.Parameters;

internal static class Utils
{
    private static bool MatchNodeName(string node_name, string node_fqn)
    {
        // Update the regular expression ["/*" -> "(/\\w+)" and "/**" -> "(/\\w+)*"]
        var regex = Regex.Replace(node_name, "/*", "(/\\w+)");
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

        foreach(var (k,v) in overrides)
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
                var initial_map = FromRclParams(param, nodeFqn);
                if (initial_map.TryGetValue(nodeFqn, out var dict))
                {
                    foreach (var (k, v) in dict)
                    {
                        result[k] = v;
                    }
                }
            }
            finally
            {
                rcl_yaml_node_struct_fini(param);
            }
        }
    }

    private static unsafe Dictionary<string, ParameterDictionary> FromRclParams(rcl_params_t* c_params, string nodeFqn = "")
    {
        var dest = new Dictionary<string, ParameterDictionary>();
        for (int i = 0; i < (int)c_params->num_nodes.Value; i++)
        {
            var node_name = StringMarshal.CreatePooledString(c_params->node_names[i])!;
            if (!node_name.StartsWith('/'))
            {
                node_name = "/" + node_name;
            }

            if (nodeFqn != "")
            {
                if (!MatchNodeName(node_name, nodeFqn))
                {
                    continue;
                }

                node_name = nodeFqn;
            }

            if (!dest.TryGetValue(node_name, out var param_node))
            {
                param_node = new ParameterDictionary();
                dest[node_name] = param_node;
            }

            rcl_node_params_t* c_params_node = &(c_params->@params[i]);

            for (int p = 0; p < (int)c_params_node->num_params.Value; ++p)
            {
                var c_param_name = StringMarshal.CreatePooledString(c_params_node->parameter_names[p]);
                if (c_param_name != null)
                {
                    var c_param_value = &(c_params_node->parameter_values[p]);
                    param_node[c_param_name] = ConvertVariant(c_param_value);
                }
            }
        }

        return dest;

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
