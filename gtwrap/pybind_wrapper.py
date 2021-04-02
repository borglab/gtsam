#!/usr/bin/env python3
"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Code generator for wrapping a C++ module with Pybind11
Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=too-many-arguments, too-many-instance-attributes, no-self-use, no-else-return, too-many-arguments, unused-format-string-argument, line-too-long

import re

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator


class PybindWrapper:
    """
    Class to generate binding code for Pybind11 specifically.
    """
    def __init__(self,
                 module,
                 module_name,
                 top_module_namespaces='',
                 use_boost=False,
                 ignore_classes=(),
                 module_template=""):
        self.module = module
        self.module_name = module_name
        self.top_module_namespaces = top_module_namespaces
        self.use_boost = use_boost
        self.ignore_classes = ignore_classes
        self._serializing_classes = list()
        self.module_template = module_template
        self.python_keywords = ['print', 'lambda']

        # amount of indentation to add before each function/method declaration.
        self.method_indent = '\n' + (' ' * 8)

    def _py_args_names(self, args_list):
        """Set the argument names in Pybind11 format."""
        names = args_list.args_names()
        if names:
            py_args = ['py::arg("{}")'.format(name) for name in names]
            return ", " + ", ".join(py_args)
        else:
            return ''

    def _method_args_signature_with_names(self, args_list):
        """Define the method signature types with the argument names."""
        cpp_types = args_list.to_cpp(self.use_boost)
        names = args_list.args_names()
        types_names = ["{} {}".format(ctype, name) for ctype, name in zip(cpp_types, names)]

        return ', '.join(types_names)

    def wrap_ctors(self, my_class):
        """Wrap the constructors."""
        res = ""
        for ctor in my_class.ctors:
            res += (self.method_indent + '.def(py::init<{args_cpp_types}>()'
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
            serialize_method = self.method_indent + \
                ".def(\"serialize\", []({class_inst} self){{ return gtsam::serialize(*self); }})".format(class_inst=cpp_class + '*')
            deserialize_method = self.method_indent + \
                     ".def(\"deserialize\", []({class_inst} self, string serialized){{ gtsam::deserialize(serialized, *self); }}, py::arg(\"serialized\"))" \
                       .format(class_inst=cpp_class + '*')
            return serialize_method + deserialize_method

        if cpp_method == "pickle":
            if not cpp_class in self._serializing_classes:
                raise ValueError("Cannot pickle a class which is not serializable")
            pickle_method = self.method_indent + \
                ".def(py::pickle({indent}    [](const {cpp_class} &a){{ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }},{indent}    [](py::tuple t){{ /* __setstate__ */ {cpp_class} obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }}))"
            return pickle_method.format(cpp_class=cpp_class, indent=self.method_indent)

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
                   py_method=py_method if not py_method in self.python_keywords
                   else py_method + "_",
                   opt_self="{cpp_class}* self".format(
                       cpp_class=cpp_class) if is_method else "",
                   opt_comma=', ' if is_method and args_names else '',
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
        """
        Wrap all the methods in the `cpp_class`.

        This function is also used to wrap global functions.
        """
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
        """Wrap all the properties in the `cpp_class`."""
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

    def wrap_operators(self, operators, cpp_class, prefix='\n' + ' ' * 8):
        """Wrap all the overloaded operators in the `cpp_class`."""
        res = ""
        template = "{prefix}.def({{0}})".format(prefix=prefix)
        for op in operators:
            if op.operator == "[]":  # __getitem__
                res += "{prefix}.def(\"__getitem__\", &{cpp_class}::operator[])".format(
                    prefix=prefix, cpp_class=cpp_class)
            elif op.operator == "()":  # __call__
                res += "{prefix}.def(\"__call__\", &{cpp_class}::operator())".format(
                    prefix=prefix, cpp_class=cpp_class)
            elif op.is_unary:
                res += template.format("{0}py::self".format(op.operator))
            else:
                res += template.format("py::self {0} py::self".format(
                    op.operator))
        return res

    def wrap_instantiated_class(self, instantiated_class):
        """Wrap the class."""
        module_var = self._gen_module_var(instantiated_class.namespaces())
        cpp_class = instantiated_class.cpp_class()
        if cpp_class in self.ignore_classes:
            return ""
        return (
            '\n    py::class_<{cpp_class}, {class_parent}'
            '{shared_ptr_type}::shared_ptr<{cpp_class}>>({module_var}, "{class_name}")'
            '{wrapped_ctors}'
            '{wrapped_methods}'
            '{wrapped_static_methods}'
            '{wrapped_properties}'
            '{wrapped_operators};\n'.format(
                shared_ptr_type=('boost' if self.use_boost else 'std'),
                cpp_class=cpp_class,
                class_name=instantiated_class.name,
                class_parent="{instantiated_class.parent_class}, ".format(
                    instantiated_class=instantiated_class)
                if instantiated_class.parent_class else '',
                module_var=module_var,
                wrapped_ctors=self.wrap_ctors(instantiated_class),
                wrapped_methods=self.wrap_methods(instantiated_class.methods,
                                                  cpp_class),
                wrapped_static_methods=self.wrap_methods(
                    instantiated_class.static_methods, cpp_class),
                wrapped_properties=self.wrap_properties(
                    instantiated_class.properties, cpp_class),
                wrapped_operators=self.wrap_operators(
                    instantiated_class.operators, cpp_class)))

    def wrap_stl_class(self, stl_class):
        """Wrap STL containers."""
        module_var = self._gen_module_var(stl_class.namespaces())
        cpp_class = stl_class.cpp_class()
        if cpp_class in self.ignore_classes:
            return ""

        return (
            '\n    py::class_<{cpp_class}, {class_parent}'
            '{shared_ptr_type}::shared_ptr<{cpp_class}>>({module_var}, "{class_name}")'
            '{wrapped_ctors}'
            '{wrapped_methods}'
            '{wrapped_static_methods}'
            '{wrapped_properties};\n'.format(
                shared_ptr_type=('boost' if self.use_boost else 'std'),
                cpp_class=cpp_class,
                class_name=stl_class.name,
                class_parent=str(stl_class.parent_class) +
                (', ' if stl_class.parent_class else ''),
                module_var=module_var,
                wrapped_ctors=self.wrap_ctors(stl_class),
                wrapped_methods=self.wrap_methods(stl_class.methods,
                                                  cpp_class),
                wrapped_static_methods=self.wrap_methods(
                    stl_class.static_methods, cpp_class),
                wrapped_properties=self.wrap_properties(
                    stl_class.properties, cpp_class),
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
        """Wrap the complete `namespace`."""
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
            all_funcs = [
                func for func in namespace.content
                if isinstance(func, (parser.GlobalFunction,
                                     instantiator.InstantiatedGlobalFunction))
            ]
            wrapped += self.wrap_methods(
                all_funcs,
                self._add_namespaces('', namespaces)[:-2],
                prefix='\n' + ' ' * 4 + module_var,
                suffix=';',
            )
        return wrapped, includes

    def wrap(self):
        """Wrap the code in the interface file."""
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

        holder_type = "PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, " \
                      "{shared_ptr_type}::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);"
        include_boost = "#include <boost/shared_ptr.hpp>" if self.use_boost else ""

        return self.module_template.format(
            include_boost=include_boost,
            module_name=self.module_name,
            includes=includes,
            holder_type=holder_type.format(shared_ptr_type=('boost' if self.use_boost else 'std'))
            if self.use_boost else "",
            wrapped_namespace=wrapped_namespace,
            boost_class_export=boost_class_export,
        )
