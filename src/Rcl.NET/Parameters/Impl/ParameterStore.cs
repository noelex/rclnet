using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Parameters.Impl;


class ParameterStore
{
    private readonly ParameterDescriptor _descriptor;
    private Variant _value;

    public ParameterStore(ParameterDescriptor descriptor)
    {
        _descriptor = descriptor;
    }

    public Variant Value => _value;

    public ParameterDescriptor Descriptor => _descriptor;

    private ValidationResult Validate(Variant value, bool isInitializing)
    {
        if (!isInitializing && _descriptor.ReadOnly)
        {
            return ValidationResult.Failure($"Parameter '{_descriptor.Name}' is read-only.");
        }

        if (_descriptor.Type != value.Kind && !_descriptor.DynamicTyping)
        {
            return ValidationResult.Failure($"Parameter '{_descriptor.Name}' of type '{_descriptor.Type}' " +
                $"cannot be assigned with value of type '{value.Kind}'.");
        }

        if (value.Kind == ValueKind.Integer && _descriptor.IntegerRange != null)
        {
            if (!_descriptor.IntegerRange.IsInRange((long)value))
            {
                return ValidationResult.Failure($"Value of parameter '{_descriptor.Name}' is out of range.");
            }
        }

        if (value.Kind == ValueKind.Double && _descriptor.FloatingPointRange != null)
        {
            if (!_descriptor.FloatingPointRange.IsInRange((double)value))
            {
                return ValidationResult.Failure($"Value of parameter '{_descriptor.Name}' is out of range.");
            }
        }

        return ValidationResult.Success();
    }

    private ValidationResult Set(Variant value, bool isInitializing)
    {
        var result = Validate(value, isInitializing);
        if (result.IsSuccessful)
        {
            UnsafeSet(value);
        }

        return result;
    }

    public ValidationResult Validate(Variant value) => Validate(value, false);

    public ValidationResult Initialize(Variant value) => Set(value, true);

    public ValidationResult Set(Variant value) => Set(value, false);

    public void UnsafeSet(Variant value)
    {
        _value = value;
    }

}