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
from pathlib import Path
from typing import List

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator


class PybindWrapper:
    """
    Class to generate binding code for Pybind11 specifically.
    """
    def __init__(self,
                 module_name,
                 top_module_namespaces='',
                 use_boost=False,
                 ignore_classes=(),
                 module_template=""):
        self.module_name = module_name
        self.top_module_namespaces = top_module_namespaces
        self.use_boost = use_boost
        self.ignore_classes = ignore_classes
        self._serializing_classes = []
        self.module_template = module_template
        self.python_keywords = [
            'lambda', 'False', 'def', 'if', 'raise', 'None', 'del', 'import',
            'return', 'True', 'elif', 'in', 'try', 'and', 'else', 'is',
            'while', 'as', 'except', 'lambda', 'with', 'assert', 'finally',
            'nonlocal', 'yield', 'break', 'for', 'not', 'class', 'from', 'or',
            'continue', 'global', 'pass'
        ]

        # amount of indentation to add before each function/method declaration.
        self.method_indent = '\n' + (' ' * 8)

        # Special methods which are leveraged by ipython/jupyter notebooks
        self._ipython_special_methods = [
            "svg", "png", "jpeg", "html", "javascript", "markdown", "latex"
        ]

    def _py_args_names(self, args):
        """Set the argument names in Pybind11 format."""
        names = args.names()
        if names:
            py_args = []
            for arg in args.list():
                if arg.default is not None:
                    default = ' = {arg.default}'.format(arg=arg)
                else:
                    default = ''
                argument = 'py::arg("{name}"){default}'.format(
                    name=arg.name, default='{0}'.format(default))
                py_args.append(argument)
            return ", " + ", ".join(py_args)
        else:
            return ''

    def _method_args_signature(self, args):
        """Generate the argument types and names as per the method signature."""
        cpp_types = args.to_cpp(self.use_boost)
        names = args.names()
        types_names = [
            "{} {}".format(ctype, name)
            for ctype, name in zip(cpp_types, names)
        ]

        return ', '.join(types_names)

    def wrap_ctors(self, my_class):
        """Wrap the constructors."""
        res = ""
        for ctor in my_class.ctors:
            res += (
                self.method_indent + '.def(py::init<{args_cpp_types}>()'
                '{py_args_names})'.format(
                    args_cpp_types=", ".join(ctor.args.to_cpp(self.use_boost)),
                    py_args_names=self._py_args_names(ctor.args),
                ))
        return res

    def _wrap_serialization(self, cpp_class):
        """Helper method to add serialize, deserialize and pickle methods to the wrapped class."""
        if not cpp_class in self._serializing_classes:
            self._serializing_classes.append(cpp_class)

        serialize_method = self.method_indent + \
            ".def(\"serialize\", []({class_inst} self){{ return gtsam::serialize(*self); }})".format(class_inst=cpp_class + '*')

        deserialize_method = self.method_indent + \
                    '.def("deserialize", []({class_inst} self, string serialized)' \
                    '{{ gtsam::deserialize(serialized, *self); }}, py::arg("serialized"))' \
                    .format(class_inst=cpp_class + '*')

        # Since this class supports serialization, we also add the pickle method.
        pickle_method = self.method_indent + \
            ".def(py::pickle({indent}    [](const {cpp_class} &a){{ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }},{indent}    [](py::tuple t){{ /* __setstate__ */ {cpp_class} obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }}))"

        return serialize_method + deserialize_method + \
            pickle_method.format(cpp_class=cpp_class, indent=self.method_indent)

    def _wrap_print(self, ret: str, method: parser.Method, cpp_class: str,
                    args_names: List[str], args_signature_with_names: str,
                    py_args_names: str, prefix: str, suffix: str):
        """
        Update the print method to print to the output stream and append a __repr__ method.

        Args:
            ret (str): The result of the parser.
            method (parser.Method): The method to be wrapped.
            cpp_class (str): The C++ name of the class to which the method belongs.
            args_names (List[str]): List of argument variable names passed to the method.
            args_signature_with_names (str): C++ arguments containing their names and type signatures.
            py_args_names (str): The pybind11 formatted version of the argument list.
            prefix (str): Prefix to add to the wrapped method when writing to the cpp file.
            suffix (str): Suffix to add to the wrapped method when writing to the cpp file.

        Returns:
            str: The wrapped print method.
        """
        # Redirect stdout - see pybind docs for why this is a good idea:
        # https://pybind11.readthedocs.io/en/stable/advanced/pycpp/utilities.html#capturing-standard-output-from-ostream
        ret = ret.replace('self->print',
                          'py::scoped_ostream_redirect output; self->print')

        # Make __repr__() call .print() internally
        ret += '''{prefix}.def("__repr__",
                    [](const {cpp_class}& self{opt_comma}{args_signature_with_names}){{
                        gtsam::RedirectCout redirect;
                        self.{method_name}({method_args});
                        return redirect.str();
                    }}{py_args_names}){suffix}'''.format(
            prefix=prefix,
            cpp_class=cpp_class,
            opt_comma=', ' if args_names else '',
            args_signature_with_names=args_signature_with_names,
            method_name=method.name,
            method_args=", ".join(args_names) if args_names else '',
            py_args_names=py_args_names,
            suffix=suffix)
        return ret

    def _wrap_method(self,
                     method,
                     cpp_class,
                     prefix,
                     suffix,
                     method_suffix=""):
        """
        Wrap the `method` for the class specified by `cpp_class`.

        Args:
            method: The method to wrap.
            cpp_class: The C++ name of the class to which the method belongs.
            prefix: Prefix to add to the wrapped method when writing to the cpp file.
            suffix: Suffix to add to the wrapped method when writing to the cpp file.
            method_suffix: A string to append to the wrapped method name.
        """
        py_method = method.name + method_suffix
        cpp_method = method.to_cpp()

        args_names = method.args.names()
        py_args_names = self._py_args_names(method.args)
        args_signature_with_names = self._method_args_signature(method.args)

        # Special handling for the serialize/serializable method
        if cpp_method in ["serialize", "serializable"]:
            return self._wrap_serialization(cpp_class)

        # Special handling of ipython specific methods
        # https://ipython.readthedocs.io/en/stable/config/integrating.html
        if cpp_method in self._ipython_special_methods:
            idx = self._ipython_special_methods.index(cpp_method)
            py_method = f"_repr_{self._ipython_special_methods[idx]}_"

        # Add underscore to disambiguate if the method name matches a python keyword
        if py_method in self.python_keywords:
            py_method = py_method + "_"

        is_method = isinstance(
            method, (parser.Method, instantiator.InstantiatedMethod))
        is_static = isinstance(
            method,
            (parser.StaticMethod, instantiator.InstantiatedStaticMethod))
        return_void = method.return_type.is_void()

        caller = cpp_class + "::" if not is_method else "self->"
        function_call = ('{opt_return} {caller}{method_name}'
                         '({args_names});'.format(
                             opt_return='return' if not return_void else '',
                             caller=caller,
                             method_name=cpp_method,
                             args_names=', '.join(args_names),
                         ))

        ret = ('{prefix}.{cdef}("{py_method}",'
               '[]({opt_self}{opt_comma}{args_signature_with_names}){{'
               '{function_call}'
               '}}'
               '{py_args_names}){suffix}'.format(
                   prefix=prefix,
                   cdef="def_static" if is_static else "def",
                   py_method=py_method,
                   opt_self="{cpp_class}* self".format(
                       cpp_class=cpp_class) if is_method else "",
                   opt_comma=', ' if is_method and args_names else '',
                   args_signature_with_names=args_signature_with_names,
                   function_call=function_call,
                   py_args_names=py_args_names,
                   suffix=suffix,
               ))

        # Create __repr__ override
        # We allow all arguments to .print() and let the compiler handle type mismatches.
        if method.name == 'print':
            ret = self._wrap_print(ret, method, cpp_class, args_names,
                                   args_signature_with_names, py_args_names,
                                   prefix, suffix)

        return ret

    def wrap_methods(self,
                     methods,
                     cpp_class,
                     prefix='\n' + ' ' * 8,
                     suffix=''):
        """
        Wrap all the methods in the `cpp_class`.
        """
        res = ""
        for method in methods:

            # To avoid type confusion for insert
            if method.name == 'insert' and cpp_class == 'gtsam::Values':
                name_list = method.args.names()
                type_list = method.args.to_cpp(self.use_boost)
                # inserting non-wrapped value types
                if type_list[0].strip() == 'size_t':
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

    def wrap_variable(self,
                      namespace,
                      module_var,
                      variable,
                      prefix='\n' + ' ' * 8):
        """
        Wrap a variable that's not part of a class (i.e. global)
        """
        variable_value = ""
        if variable.default is None:
            variable_value = variable.name
        else:
            variable_value = variable.default

        return '{prefix}{module_var}.attr("{variable_name}") = {namespace}{variable_value};'.format(
            prefix=prefix,
            module_var=module_var,
            variable_name=variable.name,
            namespace=namespace,
            variable_value=variable_value)

    def wrap_properties(self, properties, cpp_class, prefix='\n' + ' ' * 8):
        """Wrap all the properties in the `cpp_class`."""
        res = ""
        for prop in properties:
            res += ('{prefix}.def_{property}("{property_name}", '
                    '&{cpp_class}::{property_name})'.format(
                        prefix=prefix,
                        property="readonly"
                        if prop.ctype.is_const else "readwrite",
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

    def wrap_enum(self, enum, class_name='', module=None, prefix=' ' * 4):
        """
        Wrap an enum.

        Args:
            enum: The parsed enum to wrap.
            class_name: The class under which the enum is defined.
            prefix: The amount of indentation.
        """
        if module is None:
            module = self._gen_module_var(enum.namespaces())

        cpp_class = enum.cpp_typename().to_cpp()
        if class_name:
            # If class_name is provided, add that as the namespace
            cpp_class = class_name + "::" + cpp_class

        res = '{prefix}py::enum_<{cpp_class}>({module}, "{enum.name}", py::arithmetic())'.format(
            prefix=prefix, module=module, enum=enum, cpp_class=cpp_class)
        for enumerator in enum.enumerators:
            res += '\n{prefix}    .value("{enumerator.name}", {cpp_class}::{enumerator.name})'.format(
                prefix=prefix, enumerator=enumerator, cpp_class=cpp_class)
        res += ";\n\n"
        return res

    def wrap_enums(self, enums, instantiated_class, prefix=' ' * 4):
        """Wrap multiple enums defined in a class."""
        cpp_class = instantiated_class.to_cpp()
        module_var = instantiated_class.name.lower()
        res = ''

        for enum in enums:
            res += "\n" + self.wrap_enum(
                enum, class_name=cpp_class, module=module_var, prefix=prefix)
        return res

    def wrap_instantiated_class(
            self, instantiated_class: instantiator.InstantiatedClass):
        """Wrap the class."""
        module_var = self._gen_module_var(instantiated_class.namespaces())
        cpp_class = instantiated_class.to_cpp()
        if cpp_class in self.ignore_classes:
            return ""
        if instantiated_class.parent_class:
            class_parent = "{instantiated_class.parent_class}, ".format(
                instantiated_class=instantiated_class)
        else:
            class_parent = ''

        if instantiated_class.enums:
            # If class has enums, define an instance and set module_var to the instance
            instance_name = instantiated_class.name.lower()
            class_declaration = (
                '\n    py::class_<{cpp_class}, {class_parent}'
                '{shared_ptr_type}::shared_ptr<{cpp_class}>> '
                '{instance_name}({module_var}, "{class_name}");'
                '\n    {instance_name}').format(
                    shared_ptr_type=('boost' if self.use_boost else 'std'),
                    cpp_class=cpp_class,
                    class_name=instantiated_class.name,
                    class_parent=class_parent,
                    instance_name=instance_name,
                    module_var=module_var)
            module_var = instance_name

        else:
            class_declaration = (
                '\n    py::class_<{cpp_class}, {class_parent}'
                '{shared_ptr_type}::shared_ptr<{cpp_class}>>({module_var}, "{class_name}")'
            ).format(shared_ptr_type=('boost' if self.use_boost else 'std'),
                     cpp_class=cpp_class,
                     class_name=instantiated_class.name,
                     class_parent=class_parent,
                     module_var=module_var)

        return ('{class_declaration}'
                '{wrapped_ctors}'
                '{wrapped_methods}'
                '{wrapped_static_methods}'
                '{wrapped_properties}'
                '{wrapped_operators};\n'.format(
                    class_declaration=class_declaration,
                    wrapped_ctors=self.wrap_ctors(instantiated_class),
                    wrapped_methods=self.wrap_methods(
                        instantiated_class.methods, cpp_class),
                    wrapped_static_methods=self.wrap_methods(
                        instantiated_class.static_methods, cpp_class),
                    wrapped_properties=self.wrap_properties(
                        instantiated_class.properties, cpp_class),
                    wrapped_operators=self.wrap_operators(
                        instantiated_class.operators, cpp_class)))

    def wrap_instantiated_declaration(
            self, instantiated_decl: instantiator.InstantiatedDeclaration):
        """Wrap the forward declaration."""
        module_var = self._gen_module_var(instantiated_decl.namespaces())
        cpp_class = instantiated_decl.to_cpp()
        if cpp_class in self.ignore_classes:
            return ""

        res = (
            '\n    py::class_<{cpp_class}, '
            '{shared_ptr_type}::shared_ptr<{cpp_class}>>({module_var}, "{class_name}");'
        ).format(shared_ptr_type=('boost' if self.use_boost else 'std'),
                 cpp_class=cpp_class,
                 class_name=instantiated_decl.name,
                 module_var=module_var)
        return res

    def wrap_stl_class(self, stl_class):
        """Wrap STL containers."""
        module_var = self._gen_module_var(stl_class.namespaces())
        cpp_class = stl_class.to_cpp()
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

    def wrap_functions(self,
                       functions,
                       namespace,
                       prefix='\n' + ' ' * 8,
                       suffix=''):
        """
        Wrap all the global functions.
        """
        res = ""
        for function in functions:

            function_name = function.name

            # Add underscore to disambiguate if the function name matches a python keyword
            python_keywords = self.python_keywords + ['print']
            if function_name in python_keywords:
                function_name = function_name + "_"

            cpp_method = function.to_cpp()

            is_static = isinstance(function, parser.StaticMethod)
            return_void = function.return_type.is_void()
            args_names = function.args.names()
            py_args_names = self._py_args_names(function.args)
            args_signature = self._method_args_signature(function.args)

            caller = namespace + "::"
            function_call = ('{opt_return} {caller}{function_name}'
                             '({args_names});'.format(
                                 opt_return='return'
                                 if not return_void else '',
                                 caller=caller,
                                 function_name=cpp_method,
                                 args_names=', '.join(args_names),
                             ))

            ret = ('{prefix}.{cdef}("{function_name}",'
                   '[]({args_signature}){{'
                   '{function_call}'
                   '}}'
                   '{py_args_names}){suffix}'.format(
                       prefix=prefix,
                       cdef="def_static" if is_static else "def",
                       function_name=function_name,
                       args_signature=args_signature,
                       function_call=function_call,
                       py_args_names=py_args_names,
                       suffix=suffix))

            res += ret

        return res

    def _partial_match(self, namespaces1, namespaces2):
        for i in range(min(len(namespaces1), len(namespaces2))):
            if namespaces1[i] != namespaces2[i]:
                return False
        return True

    def _gen_module_var(self, namespaces):
        """Get the Pybind11 module name from the namespaces."""
        # We skip the first value in namespaces since it is empty
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
                    include = "{}\n".format(element)
                    # replace the angle brackets with quotes
                    include = include.replace('<', '"').replace('>', '"')
                    includes += include
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
                wrapped += (
                    ' ' * 4 + 'pybind11::module {module_var} = '
                    '{parent_module_var}.def_submodule("{namespace}", "'
                    '{namespace} submodule");\n'.format(
                        module_var=module_var,
                        namespace=namespace.name,
                        parent_module_var=self._gen_module_var(
                            namespaces[:-1]),
                    ))

            # Wrap an include statement, namespace, class or enum
            for element in namespace.content:
                if isinstance(element, parser.Include):
                    include = "{}\n".format(element)
                    # replace the angle brackets with quotes
                    include = include.replace('<', '"').replace('>', '"')
                    includes += include
                elif isinstance(element, parser.Namespace):
                    wrapped_namespace, includes_namespace = self.wrap_namespace(
                        element)
                    wrapped += wrapped_namespace
                    includes += includes_namespace

                elif isinstance(element, instantiator.InstantiatedClass):
                    wrapped += self.wrap_instantiated_class(element)
                    wrapped += self.wrap_enums(element.enums, element)

                elif isinstance(element, instantiator.InstantiatedDeclaration):
                    wrapped += self.wrap_instantiated_declaration(element)

                elif isinstance(element, parser.Variable):
                    variable_namespace = self._add_namespaces('', namespaces)
                    wrapped += self.wrap_variable(namespace=variable_namespace,
                                                  module_var=module_var,
                                                  variable=element,
                                                  prefix='\n' + ' ' * 4)

                elif isinstance(element, parser.Enum):
                    wrapped += self.wrap_enum(element)

            # Global functions.
            all_funcs = [
                func for func in namespace.content
                if isinstance(func, (parser.GlobalFunction,
                                     instantiator.InstantiatedGlobalFunction))
            ]
            wrapped += self.wrap_functions(
                all_funcs,
                self._add_namespaces('', namespaces)[:-2],
                prefix='\n' + ' ' * 4 + module_var,
                suffix=';',
            )
        return wrapped, includes

    def wrap_file(self, content, module_name=None, submodules=None):
        """
        Wrap the code in the interface file.

        Args:
            content: The contents of the interface file.
            module_name: The name of the module.
            submodules: List of other interface file names that should be linked to.
        """
        # Parse the contents of the interface file
        module = parser.Module.parseString(content)
        # Instantiate all templates
        module = instantiator.instantiate_namespace(module)

        wrapped_namespace, includes = self.wrap_namespace(module)

        # Export classes for serialization.
        boost_class_export = ""
        for cpp_class in self._serializing_classes:
            new_name = cpp_class
            # The boost's macro doesn't like commas, so we have to typedef.
            if ',' in cpp_class:
                new_name = re.sub("[,:<> ]", "", cpp_class)
                boost_class_export += "typedef {cpp_class} {new_name};\n".format(  # noqa
                    cpp_class=cpp_class, new_name=new_name)

            boost_class_export += "BOOST_CLASS_EXPORT({new_name})\n".format(
                new_name=new_name, )

        # Reset the serializing classes list
        self._serializing_classes = []

        holder_type = "PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, " \
                      "{shared_ptr_type}::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);"
        include_boost = "#include <boost/shared_ptr.hpp>" if self.use_boost else ""

        submodules_init = []

        if submodules is not None:
            module_def = "PYBIND11_MODULE({0}, m_)".format(module_name)

            for idx, submodule in enumerate(submodules):
                submodules[idx] = "void {0}(py::module_ &);".format(submodule)
                submodules_init.append("{0}(m_);".format(submodule))

        else:
            module_def = "void {0}(py::module_ &m_)".format(module_name)
            submodules = []

        return self.module_template.format(
            include_boost=include_boost,
            module_def=module_def,
            module_name=module_name,
            includes=includes,
            holder_type=holder_type.format(
                shared_ptr_type=('boost' if self.use_boost else 'std'))
            if self.use_boost else "",
            wrapped_namespace=wrapped_namespace,
            boost_class_export=boost_class_export,
            submodules="\n".join(submodules),
            submodules_init="\n".join(submodules_init),
        )

    def wrap_submodule(self, source):
        """
        Wrap a list of submodule files, i.e. a set of interface files which are
        in support of a larger wrapping project.

        E.g. This is used in GTSAM where we have a main gtsam.i, but various smaller .i files
        which are the submodules.
        The benefit of this scheme is that it reduces compute and memory usage during compilation.

        Args:
            source: Interface file which forms the submodule.
        """
        filename = Path(source).name
        module_name = Path(source).stem

        # Read in the complete interface (.i) file
        with open(source, "r") as f:
            content = f.read()
        # Wrap the read-in content
        cc_content = self.wrap_file(content, module_name=module_name)

        # Generate the C++ code which Pybind11 will use.
        with open(filename.replace(".i", ".cpp"), "w") as f:
            f.write(cc_content)

    def wrap(self, sources, main_module_name):
        """
        Wrap all the main interface file.

        Args:
            sources: List of all interface files.
                The first file should be the main module.
            main_module_name: The name for the main module.
        """
        main_module = sources[0]

        # Get all the submodule names.
        submodules = []
        for source in sources[1:]:
            module_name = Path(source).stem
            submodules.append(module_name)

        with open(main_module, "r") as f:
            content = f.read()
        cc_content = self.wrap_file(content,
                                    module_name=self.module_name,
                                    submodules=submodules)

        # Generate the C++ code which Pybind11 will use.
        with open(main_module_name, "w") as f:
            f.write(cc_content)
