#!/usr/bin/env python3
"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Code generator for wrapping a C++ module with Pybind11
Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar and Frank Dellaert
"""
import argparse
import re
import textwrap

import interface_parser as parser
import template_instantiator as instantiator


class PybindWrapper(object):
    def __init__(self,
                 module,
                 module_name,
                 top_module_namespaces='',
                 use_boost=False,
                 ignore_classes=[],
                 module_template=""):
        self.module = module
        self.module_name = module_name
        self.top_module_namespaces = top_module_namespaces
        self.use_boost = use_boost
        self.ignore_classes = ignore_classes
        self._serializing_classes = list()
        self.module_template = module_template
        self.python_keywords = ['print', 'lambda']

    def _py_args_names(self, args_list):
        names = args_list.args_names()
        if names:
            py_args = ['py::arg("{}")'.format(name) for name in names]
            return ", " + ", ".join(py_args)
        else:
            return ''

    def _method_args_signature_with_names(self, args_list):
        cpp_types = args_list.to_cpp(self.use_boost)
        names = args_list.args_names()
        types_names = ["{} {}".format(ctype, name) for ctype, name in zip(cpp_types, names)]

        return ','.join(types_names)

    def wrap_ctors(self, my_class):
        res = ""
        for ctor in my_class.ctors:
            res += ('\n' + ' ' * 8 + '.def(py::init<{args_cpp_types}>()'
                    '{py_args_names})'.format(
                        args_cpp_types=", ".join(ctor.args.to_cpp(self.use_boost)),
                        py_args_names=self._py_args_names(ctor.args),
                    ))
        return res

    def _wrap_method(self, method, cpp_class, prefix, suffix, method_suffix=""):
        py_method = method.name + method_suffix
        cpp_method = method.to_cpp()

        if cpp_method in ["serialize", "serializable"]:
            if not cpp_class in self._serializing_classes:
                self._serializing_classes.append(cpp_class)
            return textwrap.dedent('''
                    .def("serialize",
                        []({class_inst} self){{
                            return gtsam::serialize(*self);
                        }}
                    )
                    .def("deserialize",
                        []({class_inst} self, string serialized){{
                            gtsam::deserialize(serialized, *self);
                        }}, py::arg("serialized"))
                    '''.format(class_inst=cpp_class + '*'))

        is_method = isinstance(method, instantiator.InstantiatedMethod)
        is_static = isinstance(method, parser.StaticMethod)
        return_void = method.return_type.is_void()
        args_names = method.args.args_names()
        py_args_names = self._py_args_names(method.args)
        args_signature_with_names = self._method_args_signature_with_names(method.args)

        caller = cpp_class + "::" if not is_method else "self->"
        function_call = ('{opt_return} {caller}{function_name}'
                         '({args_names});'.format(
                             opt_return='return' if not return_void else '',
                             caller=caller,
                             function_name=cpp_method,
                             args_names=', '.join(args_names),
                         ))

        ret = ('{prefix}.{cdef}("{py_method}",'
               '[]({opt_self}{opt_comma}{args_signature_with_names}){{'
               '{function_call}'
               '}}'
               '{py_args_names}){suffix}'.format(
                   prefix=prefix,
                   cdef="def_static" if is_static else "def",
                   py_method=py_method if not py_method in self.python_keywords else py_method + "_",
                   opt_self="{cpp_class}* self".format(cpp_class=cpp_class) if is_method else "",
                   cpp_class=cpp_class,
                   cpp_method=cpp_method,
                   opt_comma=',' if is_method and args_names else '',
                   args_signature_with_names=args_signature_with_names,
                   function_call=function_call,
                   py_args_names=py_args_names,
                   suffix=suffix,
               ))
        if method.name == 'print':
            type_list = method.args.to_cpp(self.use_boost)
            if len(type_list) > 0 and type_list[0].strip() == 'string':
                ret += '''{prefix}.def("__repr__",
                    [](const {cpp_class} &a) {{
                        gtsam::RedirectCout redirect;
                        a.print("");
                        return redirect.str();
                    }}){suffix}'''.format(
                    prefix=prefix,
                    cpp_class=cpp_class,
                    suffix=suffix,
                )
            else:
                ret += '''{prefix}.def("__repr__",
                    [](const {cpp_class} &a) {{
                        gtsam::RedirectCout redirect;
                        a.print();
                        return redirect.str();
                    }}){suffix}'''.format(
                    prefix=prefix,
                    cpp_class=cpp_class,
                    suffix=suffix,
                )
        return ret

    def wrap_methods(self, methods, cpp_class, prefix='\n' + ' ' * 8, suffix=''):
        res = ""
        for method in methods:

            # To avoid type confusion for insert, currently unused
            if method.name == 'insert' and cpp_class == 'gtsam::Values':
                name_list = method.args.args_names()
                type_list = method.args.to_cpp(self.use_boost)
                if type_list[0].strip() == 'size_t':  # inserting non-wrapped value types
                    method_suffix = '_' + name_list[1].strip()
                    res += self._wrap_method(method=method,
                                             cpp_class=cpp_class,
                                             prefix=prefix,
                                             suffix=suffix,
                                             method_suffix=method_suffix)

            res += self._wrap_method(
                method=method,
                cpp_class=cpp_class,
                prefix=prefix,
                suffix=suffix,
            )
        return res

    def wrap_properties(self, properties, cpp_class, prefix='\n' + ' ' * 8):
        res = ""
        for prop in properties:
            res += ('{prefix}.def_{property}("{property_name}", '
                    '&{cpp_class}::{property_name})'.format(
                        prefix=prefix,
                        property="readonly" if prop.ctype.is_const else "readwrite",
                        cpp_class=cpp_class,
                        property_name=prop.name,
                    ))
        return res

    def wrap_instantiated_class(self, instantiated_class):
        module_var = self._gen_module_var(instantiated_class.namespaces())
        cpp_class = instantiated_class.cpp_class()
        if cpp_class in self.ignore_classes:
            return ""
        return ('\n    py::class_<{cpp_class}, {class_parent}'
                '{shared_ptr_type}::shared_ptr<{cpp_class}>>({module_var}, "{class_name}")'
                '{wrapped_ctors}'
                '{wrapped_methods}'
                '{wrapped_static_methods}'
                '{wrapped_properties};\n'.format(
                    shared_ptr_type=('boost' if self.use_boost else 'std'),
                    cpp_class=cpp_class,
                    class_name=instantiated_class.name,
                    class_parent=str(instantiated_class.parent_class) +
                    (', ' if instantiated_class.parent_class else ''),
                    module_var=module_var,
                    wrapped_ctors=self.wrap_ctors(instantiated_class),
                    wrapped_methods=self.wrap_methods(instantiated_class.methods, cpp_class),
                    wrapped_static_methods=self.wrap_methods(instantiated_class.static_methods, cpp_class),
                    wrapped_properties=self.wrap_properties(instantiated_class.properties, cpp_class),
                ))

    def wrap_stl_class(self, stl_class):
        module_var = self._gen_module_var(stl_class.namespaces())
        cpp_class = stl_class.cpp_class()
        if cpp_class in self.ignore_classes:
            return ""

        return ('\n    py::class_<{cpp_class}, {class_parent}'
                '{shared_ptr_type}::shared_ptr<{cpp_class}>>({module_var}, "{class_name}")'
                '{wrapped_ctors}'
                '{wrapped_methods}'
                '{wrapped_static_methods}'
                '{wrapped_properties};\n'.format(
                    shared_ptr_type=('boost' if self.use_boost else 'std'),
                    cpp_class=cpp_class,
                    class_name=stl_class.name,
                    class_parent=str(stl_class.parent_class) + (', ' if stl_class.parent_class else ''),
                    module_var=module_var,
                    wrapped_ctors=self.wrap_ctors(stl_class),
                    wrapped_methods=self.wrap_methods(stl_class.methods, cpp_class),
                    wrapped_static_methods=self.wrap_methods(stl_class.static_methods, cpp_class),
                    wrapped_properties=self.wrap_properties(stl_class.properties, cpp_class),
                ))

    def _partial_match(self, namespaces1, namespaces2):
        for i in range(min(len(namespaces1), len(namespaces2))):
            if namespaces1[i] != namespaces2[i]:
                return False
        return True

    def _gen_module_var(self, namespaces):
        sub_module_namespaces = namespaces[len(self.top_module_namespaces):]
        return "m_{}".format('_'.join(sub_module_namespaces))

    def _add_namespaces(self, name, namespaces):
        if namespaces:
            # Ignore the first empty global namespace.
            idx = 1 if not namespaces[0] else 0
            return '::'.join(namespaces[idx:] + [name])
        else:
            return name

    def wrap_namespace(self, namespace):
        wrapped = ""
        includes = ""

        namespaces = namespace.full_namespaces()
        if not self._partial_match(namespaces, self.top_module_namespaces):
            return "", ""

        if len(namespaces) < len(self.top_module_namespaces):
            for element in namespace.content:
                if isinstance(element, parser.Include):
                    includes += ("{}\n".format(element).replace('<', '"').replace('>', '"'))
                if isinstance(element, parser.Namespace):
                    (
                        wrapped_namespace,
                        includes_namespace,
                    ) = self.wrap_namespace(  # noqa
                        element)
                    wrapped += wrapped_namespace
                    includes += includes_namespace
        else:
            module_var = self._gen_module_var(namespaces)

            if len(namespaces) > len(self.top_module_namespaces):
                wrapped += (' ' * 4 + 'pybind11::module {module_var} = '
                            '{parent_module_var}.def_submodule("{namespace}", "'
                            '{namespace} submodule");\n'.format(
                                module_var=module_var,
                                namespace=namespace.name,
                                parent_module_var=self._gen_module_var(namespaces[:-1]),
                            ))

            for element in namespace.content:
                if isinstance(element, parser.Include):
                    includes += ("{}\n".format(element).replace('<', '"').replace('>', '"'))
                elif isinstance(element, parser.Namespace):
                    (
                        wrapped_namespace,
                        includes_namespace,
                    ) = self.wrap_namespace(  # noqa
                        element)
                    wrapped += wrapped_namespace
                    includes += includes_namespace
                elif isinstance(element, instantiator.InstantiatedClass):
                    wrapped += self.wrap_instantiated_class(element)

            # Global functions.
            all_funcs = [func for func in namespace.content if isinstance(func, parser.GlobalFunction)]
            wrapped += self.wrap_methods(
                all_funcs,
                self._add_namespaces('', namespaces)[:-2],
                prefix='\n' + ' ' * 4 + module_var,
                suffix=';',
            )
        return wrapped, includes

    def wrap(self):
        wrapped_namespace, includes = self.wrap_namespace(self.module)

        # Export classes for serialization.
        boost_class_export = ""
        for cpp_class in self._serializing_classes:
            new_name = cpp_class
            # The boost's macro doesn't like commas, so we have to typedef.
            if ',' in cpp_class:
                new_name = re.sub("[,:<> ]", "", cpp_class)
                boost_class_export += "typedef {cpp_class} {new_name};\n".format(  # noqa
                    cpp_class=cpp_class,
                    new_name=new_name,
                )
            boost_class_export += "BOOST_CLASS_EXPORT({new_name})\n".format(new_name=new_name, )

        return self.module_template.format(
            include_boost="#include <boost/shared_ptr.hpp>" if self.use_boost else "",
            module_name=self.module_name,
            includes=includes,
            hoder_type=
            "PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, {shared_ptr_type}::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);"
            .format(shared_ptr_type=('boost' if self.use_boost else 'std')) if self.use_boost else "",
            wrapped_namespace=wrapped_namespace,
            boost_class_export=boost_class_export,
        )


def main():
    arg_parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    arg_parser.add_argument("--src", type=str, required=True, help="Input interface .h file")
    arg_parser.add_argument(
        "--module_name",
        type=str,
        required=True,
        help="Name of the Python module to be generated and "
        "used in the Python `import` statement.",
    )
    arg_parser.add_argument(
        "--out",
        type=str,
        required=True,
        help="Name of the output pybind .cc file",
    )
    arg_parser.add_argument(
        "--use-boost",
        action="store_true",
        help="using boost's shared_ptr instead of std's",
    )
    arg_parser.add_argument(
        "--top_module_namespaces",
        type=str,
        default="",
        help="C++ namespace for the top module, e.g. `ns1::ns2::ns3`. "
        "Only the content within this namespace and its sub-namespaces "
        "will be wrapped. The content of this namespace will be available at "
        "the top module level, and its sub-namespaces' in the submodules.\n"
        "For example, `import <module_name>` gives you access to a Python "
        "`<module_name>.Class` of the corresponding C++ `ns1::ns2::ns3::Class`"
        "and `from <module_name> import ns4` gives you access to a Python "
        "`ns4.Class` of the C++ `ns1::ns2::ns3::ns4::Class`. ",
    )
    arg_parser.add_argument(
        "--ignore",
        nargs='*',
        type=str,
        help="A space-separated list of classes to ignore. "
        "Class names must include their full namespaces.",
    )
    arg_parser.add_argument("--template", type=str, help="The module template file")
    args = arg_parser.parse_args()

    top_module_namespaces = args.top_module_namespaces.split("::")
    if top_module_namespaces[0]:
        top_module_namespaces = [''] + top_module_namespaces

    with open(args.src, "r") as f:
        content = f.read()
    module = parser.Module.parseString(content)
    instantiator.instantiate_namespace_inplace(module)

    with open(args.template, "r") as f:
        template_content = f.read()
    wrapper = PybindWrapper(
        module=module,
        module_name=args.module_name,
        use_boost=args.use_boost,
        top_module_namespaces=top_module_namespaces,
        ignore_classes=args.ignore,
        module_template=template_content,
    )

    cc_content = wrapper.wrap()
    with open(args.out, "w") as f:
        f.write(cc_content)


if __name__ == "__main__":
    main()
