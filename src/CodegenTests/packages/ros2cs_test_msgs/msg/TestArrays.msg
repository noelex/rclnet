string[] strings_without_defaults
string[] strings_with_defaults ["", 'hello', "world"]
string[3] strings_fixed
string[3] strings_fixed_with_defaults ["a","b","c"]
string[<=3] strings_bounded
string[<=3] strings_bounded_with_defaults ["a","b","c"]

string<=3[<=4] bounded_strings_bounded
string<=3[<=4] bounded_strings_bounded_with_defaults ["a","b","c12"]
wstring<=3[<=4] bounded_wstrings_bounded
wstring<=3[<=4] bounded_wstrings_bounded_with_defaults ["a","b","c","d34"]

int64[] integers_without_defaults
int64[] integers_with_defaults [1,2,3]
int64[3] integers_fixed
int64[3] integers_fixed_with_defaults [1,2,3]
int64[<=3] integers_bounded
int64[<=3] integers_bounded_with_defaults [1,2]

test_interface_files/BasicTypes[] basic_types
test_interface_files/BasicTypes[3] basic_types_fixed
test_interface_files/BasicTypes[<=3] basic_types_bounded

string<=3 bounded_string
string<=3 bounded_string_with_default "abc"
wstring<=3 bounded_wstring
wstring<=3 bounded_wstring_with_default "abc"