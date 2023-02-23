"""
Code to use the parsed results and convert it to a format
that Matlab's MEX compiler can use.
"""

# pylint: disable=too-many-lines, no-self-use, too-many-arguments, too-many-branches, too-many-statements, consider-using-f-string, unspecified-encoding

import copy
import os
import os.path as osp
import textwrap
from functools import partial, reduce
from typing import Dict, Iterable, List, Union

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator
from gtwrap.interface_parser.function import ArgumentList
from gtwrap.matlab_wrapper.mixins import CheckMixin, FormatMixin
from gtwrap.matlab_wrapper.templates import WrapperTemplate
from gtwrap.template_instantiator.classes import InstantiatedClass


class MatlabWrapper(CheckMixin, FormatMixin):
    """ Wrap the given C++ code into Matlab.

    Attributes
        module: the C++ module being wrapped
        module_name: name of the C++ module being wrapped
        top_module_namespace: C++ namespace for the top module (default '')
        ignore_classes: A list of classes to ignore (default [])
    """

    def __init__(self,
                 module_name,
                 top_module_namespace='',
                 ignore_classes=(),
                 use_boost_serialization=False):
        super().__init__()

        self.module_name = module_name
        self.top_module_namespace = top_module_namespace
        self.ignore_classes = ignore_classes
        self.verbose = False
        self.use_boost_serialization = use_boost_serialization

        # Map the data type to its Matlab class.
        # Found in Argument.cpp in old wrapper
        self.data_type = {
            'string': 'char',
            'char': 'char',
            'unsigned char': 'unsigned char',
            'Vector': 'double',
            'Matrix': 'double',
            'int': 'numeric',
            'size_t': 'numeric',
            'bool': 'logical'
        }
        # Map the data type into the type used in Matlab methods.
        # Found in matlab.h in old wrapper
        self.data_type_param = {
            'string': 'char',
            'char': 'char',
            'unsigned char': 'unsigned char',
            'size_t': 'int',
            'int': 'int',
            'double': 'double',
            'Point2': 'double',
            'Point3': 'double',
            'Vector': 'double',
            'Matrix': 'double',
            'bool': 'bool'
        }
        # The amount of times the wrapper has created a call to geometry_wrapper
        self.wrapper_id = 0
        # Map each wrapper id to its collector function namespace, class, type, and string format
        self.wrapper_map: Dict = {}
        # Set of all the includes in the namespace
        self.includes: List[parser.Include] = []
        # Set of all classes in the namespace
        self.classes: List[Union[parser.Class,
                                 instantiator.InstantiatedClass]] = []
        self.classes_elems: Dict[Union[parser.Class,
                                       instantiator.InstantiatedClass],
                                 int] = {}
        # Id for ordering global functions in the wrapper
        self.global_function_id = 0
        # Files and their content
        self.content: List[str] = []

        # Ensure the template file is always picked up from the correct directory.
        dir_path = osp.dirname(osp.realpath(__file__))
        with open(osp.join(dir_path, "matlab_wrapper.tpl")) as f:
            self.wrapper_file_headers = f.read()

    def add_class(self, instantiated_class):
        """Add `instantiated_class` to the list of classes."""
        if self.classes_elems.get(instantiated_class) is None:
            self.classes_elems[instantiated_class] = 0
            self.classes.append(instantiated_class)

    def _update_wrapper_id(self,
                           collector_function=None,
                           id_diff=0,
                           function_name: str = None):
        """
        Get and define wrapper ids.
        Generates the map of id -> collector function.

        Args:
            collector_function: tuple storing info about the wrapper function
                (namespace, class instance, function name, function object)
            id_diff: constant to add to the id in the map
            function_name: Optional custom function_name.

        Returns:
            the current wrapper id
        """
        if collector_function is not None:
            is_instantiated_class = isinstance(collector_function[1],
                                               instantiator.InstantiatedClass)

            if function_name is None:
                if is_instantiated_class:
                    function_name = collector_function[0] + \
                                    collector_function[1].name + '_' + collector_function[2]
                else:
                    function_name = collector_function[1].name

            self.wrapper_map[self.wrapper_id] = (
                collector_function[0], collector_function[1],
                collector_function[2], function_name + '_' +
                str(self.wrapper_id + id_diff), collector_function[3])

        self.wrapper_id += 1

        return self.wrapper_id - 1

    def _qualified_name(self, names):
        return 'handle' if names == '' else names

    def _insert_spaces(self, x, y):
        """Insert spaces at the beginning of each line

        Args:
            x: the statement currently generated
            y: the addition to add to the statement
        """
        return x + '\n' + ('' if y == '' else '  ') + y

    @staticmethod
    def _expand_default_arguments(method, save_backup=True):
        """Recursively expand all possibilities for optional default arguments.
        We create "overload" functions with fewer arguments, but since we have to "remember" what
        the default arguments are for later, we make a backup.
        """

        def args_copy(args):
            return ArgumentList([copy.copy(arg) for arg in args.list()])

        def method_copy(method):
            method2 = copy.copy(method)
            method2.args = args_copy(method.args)
            method2.args.backup = method.args.backup
            return method2

        if save_backup:
            method.args.backup = args_copy(method.args)
        method = method_copy(method)
        for arg in reversed(method.args.list()):
            if arg.default is not None:
                arg.default = None
                methodWithArg = method_copy(method)
                method.args.list().remove(arg)
                return [
                    methodWithArg,
                    *MatlabWrapper._expand_default_arguments(method,
                                                             save_backup=False)
                ]
            break
        assert all(arg.default is None for arg in method.args.list()), \
            'In parsing method {:}: Arguments with default values cannot appear before ones ' \
            'without default values.'.format(method.name)
        return [method]

    def _group_methods(self, methods):
        """Group overloaded methods together"""
        method_map = {}
        method_out = []

        for method in methods:
            method_index = method_map.get(method.name)

            if method_index is None:
                method_map[method.name] = len(method_out)
                method_out.append(
                    MatlabWrapper._expand_default_arguments(method))
            else:
                method_out[
                    method_index] += MatlabWrapper._expand_default_arguments(
                        method)

        return method_out

    def _wrap_args(self, args):
        """Wrap an interface_parser.ArgumentList into a list of arguments.

        Returns:
            A string representation of the arguments. For example:
                'int x, double y'
        """
        arg_wrap = ''

        for i, arg in enumerate(args.list(), 1):
            c_type = self._format_type_name(arg.ctype.typename,
                                            include_namespace=False)

            arg_wrap += '{c_type} {arg_name}{comma}'.format(
                c_type=c_type,
                arg_name=arg.name,
                comma='' if i == len(args.list()) else ', ')

        return arg_wrap

    def _wrap_variable_arguments(self, args, wrap_datatypes=True):
        """ Wrap an interface_parser.ArgumentList into a statement of argument
        checks.

        Returns:
            A string representation of a variable arguments for an if
            statement. For example:
                ' && isa(varargin{1},'double') && isa(varargin{2},'numeric')'
        """
        var_arg_wrap = ''

        for i, arg in enumerate(args.list(), 1):
            name = arg.ctype.typename.name
            if name in self.not_check_type:
                continue

            check_type = self.data_type_param.get(name)

            if self.data_type.get(check_type):
                check_type = self.data_type[check_type]

            if check_type is None:
                check_type = self._format_type_name(
                    arg.ctype.typename,
                    separator='.',
                    is_constructor=not wrap_datatypes)

            var_arg_wrap += " && isa(varargin{{{num}}},'{data_type}')".format(
                num=i, data_type=check_type)
            if name == 'Vector':
                var_arg_wrap += ' && size(varargin{{{num}}},2)==1'.format(
                    num=i)
            if name == 'Point2':
                var_arg_wrap += ' && size(varargin{{{num}}},1)==2'.format(
                    num=i)
                var_arg_wrap += ' && size(varargin{{{num}}},2)==1'.format(
                    num=i)
            if name == 'Point3':
                var_arg_wrap += ' && size(varargin{{{num}}},1)==3'.format(
                    num=i)
                var_arg_wrap += ' && size(varargin{{{num}}},2)==1'.format(
                    num=i)

        return var_arg_wrap

    def _wrap_list_variable_arguments(self, args):
        """ Wrap an interface_parser.ArgumentList into a list of argument
        variables.

        Returns:
            A string representation of a list of variable arguments.
            For example:
                'varargin{1}, varargin{2}, varargin{3}'
        """
        var_list_wrap = ''
        first = True

        for i in range(1, len(args.list()) + 1):
            if first:
                var_list_wrap += 'varargin{{{}}}'.format(i)
                first = False
            else:
                var_list_wrap += ', varargin{{{}}}'.format(i)

        return var_list_wrap

    def _wrap_method_check_statement(self, args: parser.ArgumentList):
        """
        Wrap the given arguments into either just a varargout call or a
        call in an if statement that checks if the parameters are accurate.

        TODO Update this method so that default arguments are supported.
        """
        arg_id = 1

        param_count = len(args)
        check_statement = 'if length(varargin) == {param_count}'.format(
            param_count=param_count)

        for _, arg in enumerate(args.list()):
            name = arg.ctype.typename.name

            if name in self.not_check_type:
                arg_id += 1
                continue

            check_type = self.data_type_param.get(name)

            if self.data_type.get(check_type):
                check_type = self.data_type[check_type]

            if check_type is None:
                check_type = self._format_type_name(arg.ctype.typename,
                                                    separator='.')

            check_statement += " && isa(varargin{{{id}}},'{ctype}')".format(
                id=arg_id, ctype=check_type)

            if name == 'Vector':
                check_statement += ' && size(varargin{{{num}}},2)==1'.format(
                    num=arg_id)
            if name == 'Point2':
                check_statement += ' && size(varargin{{{num}}},1)==2'.format(
                    num=arg_id)
                check_statement += ' && size(varargin{{{num}}},2)==1'.format(
                    num=arg_id)
            if name == 'Point3':
                check_statement += ' && size(varargin{{{num}}},1)==3'.format(
                    num=arg_id)
                check_statement += ' && size(varargin{{{num}}},2)==1'.format(
                    num=arg_id)

            arg_id += 1

        check_statement = check_statement \
            if check_statement == '' \
            else check_statement + '\n'

        return check_statement

    def _unwrap_argument(self, arg, arg_id=0, constructor=False):
        ctype_camel = self._format_type_name(arg.ctype.typename, separator='')
        ctype_sep = self._format_type_name(arg.ctype.typename)

        if self.is_ref(arg.ctype):  # and not constructor:
            arg_type = "{ctype}&".format(ctype=ctype_sep)
            unwrap = '*unwrap_shared_ptr< {ctype} >(in[{id}], "ptr_{ctype_camel}");'.format(
                ctype=ctype_sep, ctype_camel=ctype_camel, id=arg_id)

        elif self.is_ptr(arg.ctype) and \
                arg.ctype.typename.name not in self.ignore_namespace:

            arg_type = "{ctype_sep}*".format(ctype_sep=ctype_sep)
            unwrap = 'unwrap_ptr< {ctype_sep} >(in[{id}], "ptr_{ctype}");'.format(
                ctype_sep=ctype_sep, ctype=ctype_camel, id=arg_id)

        elif (self.is_shared_ptr(arg.ctype) or self.can_be_pointer(arg.ctype)) and \
                arg.ctype.typename.name not in self.ignore_namespace:

            arg_type = "std::shared_ptr<{ctype_sep}>".format(
                ctype_sep=ctype_sep)
            unwrap = 'unwrap_shared_ptr< {ctype_sep} >(in[{id}], "ptr_{ctype}");'.format(
                ctype_sep=ctype_sep, ctype=ctype_camel, id=arg_id)

        else:
            arg_type = "{ctype}".format(ctype=arg.ctype.typename.name)
            unwrap = 'unwrap< {ctype} >(in[{id}]);'.format(
                ctype=arg.ctype.typename.name, id=arg_id)

        return arg_type, unwrap

    def _wrapper_unwrap_arguments(self, args, arg_id=0, constructor=False):
        """Format the interface_parser.Arguments.

        Examples:
            ((a), unsigned char a = unwrap< unsigned char >(in[1]);),
            ((a), Test& t = *unwrap_shared_ptr< Test >(in[1], "ptr_Test");),
            ((a), std::shared_ptr<Test> p1 = unwrap_shared_ptr< Test >(in[1], "ptr_Test");)
        """
        body_args = ''

        for arg in args.list():
            arg_type, unwrap = self._unwrap_argument(arg, arg_id, constructor)

            body_args += textwrap.indent(textwrap.dedent('''\
                    {arg_type} {name} = {unwrap}
                    '''.format(arg_type=arg_type, name=arg.name,
                               unwrap=unwrap)),
                                         prefix='  ')
            arg_id += 1

        params = ''
        explicit_arg_names = [arg.name for arg in args.list()]
        # when returning the params list, we need to re-include the default args.
        for arg in args.backup.list():
            if params != '':
                params += ','

            if (arg.default is not None) and (arg.name
                                              not in explicit_arg_names):
                params += arg.default
                continue

            if not self.is_ref(arg.ctype) and (self.is_shared_ptr(arg.ctype) or \
                self.is_ptr(arg.ctype) or self.can_be_pointer(arg.ctype))and \
                    arg.ctype.typename.name not in self.ignore_namespace:
                if arg.ctype.is_shared_ptr:
                    call_type = arg.ctype.is_shared_ptr
                else:
                    call_type = arg.ctype.is_ptr
                if call_type == "":
                    params += "*"
            params += arg.name

        return params, body_args

    @staticmethod
    def _return_count(return_type):
        """The amount of objects returned by the given
        interface_parser.ReturnType.
        """
        return 1 if return_type.type2 == '' else 2

    def _wrapper_name(self):
        """Determine the name of wrapper function."""
        return self.module_name + '_wrapper'

    def class_serialize_comment(self, class_name, static_methods):
        """Generate comments for serialize methods."""
        comment_wrap = ''
        static_methods = sorted(static_methods, key=lambda name: name.name)

        for static_method in static_methods:
            if comment_wrap == '':
                comment_wrap = '%-------Static Methods-------\n'

            comment_wrap += '%{name}({args}) : returns {return_type}\n'.format(
                name=static_method.name,
                args=self._wrap_args(static_method.args),
                return_type=self._format_return_type(static_method.return_type,
                                                     include_namespace=True))

        comment_wrap += textwrap.dedent('''\
            %
            %-------Serialization Interface-------
            %string_serialize() : returns string
            %string_deserialize(string serialized) : returns {class_name}
            %
        ''').format(class_name=class_name)

        return comment_wrap

    def class_comment(self, instantiated_class):
        """Generate comments for the given class in Matlab.

        Args
            instantiated_class: the class being wrapped
            ctors: a list of the constructors in the class
            methods: a list of the methods in the class
        """
        class_name = instantiated_class.name
        ctors = instantiated_class.ctors
        properties = instantiated_class.properties
        methods = instantiated_class.methods
        static_methods = instantiated_class.static_methods

        comment = textwrap.dedent('''\
            %class {class_name}, see Doxygen page for details
            %at https://gtsam.org/doxygen/
        ''').format(class_name=class_name)

        if len(ctors) != 0:
            comment += '%\n%-------Constructors-------\n'

        # Write constructors
        for ctor in ctors:
            comment += '%{ctor_name}({args})\n'.format(ctor_name=ctor.name,
                                                       args=self._wrap_args(
                                                           ctor.args))

        if len(properties) != 0:
            comment += '%\n' \
                       '%-------Properties-------\n'
            for propty in properties:
                comment += '%{}\n'.format(propty.name)

        if len(methods) != 0:
            comment += '%\n' \
                       '%-------Methods-------\n'

        methods = sorted(methods, key=lambda name: name.name)

        # Write methods
        for method in methods:
            if method.name in self.whitelist:
                continue
            if method.name in self.ignore_methods:
                continue

            comment += '%{name}({args})'.format(name=method.name,
                                                args=self._wrap_args(
                                                    method.args))

            if method.return_type.type2 == '':
                return_type = self._format_type_name(
                    method.return_type.type1.typename)
            else:
                return_type = 'pair< {type1}, {type2} >'.format(
                    type1=self._format_type_name(
                        method.return_type.type1.typename),
                    type2=self._format_type_name(
                        method.return_type.type2.typename))

            comment += ' : returns {return_type}\n'.format(
                return_type=return_type)

        comment += '%\n'

        if len(static_methods) != 0:
            comment += self.class_serialize_comment(class_name, static_methods)

        return comment

    def wrap_method(self, methods):
        """
        Wrap methods in the body of a class.
        """
        if not isinstance(methods, list):
            methods = [methods]

        return ''

    def wrap_methods(self, methods, global_funcs=False, global_ns=None):
        """
        Wrap a sequence of methods. Groups methods with the same names
        together.
        If global_funcs is True then output every method into its own file.
        """
        output = ''
        methods = self._group_methods(methods)

        for method in methods:
            if method in self.ignore_methods:
                continue

            if global_funcs:
                method_text = self.wrap_global_function(method)
                self.content.append(("".join([
                    '+' + x + '/' for x in global_ns.full_namespaces()[1:]
                ])[:-1], [(method[0].name + '.m', method_text)]))
            else:
                method_text = self.wrap_method(method)
                output += ''

        return output

    def wrap_global_function(self, function):
        """Wrap the given global function."""
        if not isinstance(function, list):
            function = [function]

        function_name = function[0].name

        # Get all combinations of parameters
        param_wrap = ''

        for i, overload in enumerate(function):
            param_wrap += '      if' if i == 0 else '      elseif'
            param_wrap += ' length(varargin) == '

            if len(overload.args.list()) == 0:
                param_wrap += '0\n'
            else:
                param_wrap += str(len(overload.args.list())) \
                              + self._wrap_variable_arguments(overload.args, False) + '\n'

            # Determine format of return and varargout statements
            return_type_formatted = self._format_return_type(
                overload.return_type, include_namespace=True, separator=".")
            varargout = self._format_varargout(overload.return_type,
                                               return_type_formatted)

            param_wrap += textwrap.indent(textwrap.dedent('''\
                {varargout}{module_name}_wrapper({num}, varargin{{:}});
            ''').format(varargout=varargout,
                        module_name=self.module_name,
                        num=self._update_wrapper_id(
                            collector_function=(function[0].parent.name,
                                                function[i], 'global_function',
                                                None))),
                                          prefix='        ')

        param_wrap += textwrap.indent(textwrap.dedent('''\
            else
              error('Arguments do not match any overload of function {func_name}');
            end''').format(func_name=function_name),
                                      prefix='      ')

        global_function = textwrap.indent(textwrap.dedent('''\
            function varargout = {m_method}(varargin)
            {statements}
            end
        ''').format(m_method=function_name, statements=param_wrap),
                                          prefix='')

        return global_function

    def wrap_class_constructors(self, namespace_name, inst_class, parent_name,
                                ctors, is_virtual):
        """Wrap class constructor.

        Args:
            namespace_name: the name of the namespace ('' if it does not exist)
            inst_class: instance of the class
            parent_name: the name of the parent class if it exists
            ctors: the interface_parser.Constructor in the class
            is_virtual: whether the class is part of a virtual inheritance
                chain
        """
        has_parent = parent_name != ''
        class_name = inst_class.name
        if has_parent:
            parent_name = self._format_type_name(parent_name, separator=".")
        if not isinstance(ctors, Iterable):
            ctors = [ctors]

        ctors = sum((MatlabWrapper._expand_default_arguments(ctor)
                     for ctor in ctors), [])

        methods_wrap = textwrap.indent(textwrap.dedent("""\
            methods
              function obj = {class_name}(varargin)
        """).format(class_name=class_name),
                                       prefix='')

        if is_virtual:
            methods_wrap += "    if (nargin == 2 || (nargin == 3 && strcmp(varargin{3}, 'void')))"
        else:
            methods_wrap += '    if nargin == 2'

        methods_wrap += " && isa(varargin{1}, 'uint64')"
        methods_wrap += " && varargin{1} == uint64(5139824614673773682)\n"

        if is_virtual:
            methods_wrap += textwrap.indent(textwrap.dedent('''\
                if nargin == 2
                  my_ptr = varargin{{2}};
                else
                  my_ptr = {wrapper_name}({id}, varargin{{2}});
                end
            ''').format(wrapper_name=self._wrapper_name(),
                        id=self._update_wrapper_id() + 1),
                                            prefix='      ')
        else:
            methods_wrap += '      my_ptr = varargin{2};\n'

        collector_base_id = self._update_wrapper_id(
            (namespace_name, inst_class, 'collectorInsertAndMakeBase', None),
            id_diff=-1 if is_virtual else 0)

        methods_wrap += '      {ptr}{wrapper_name}({id}, my_ptr);\n' \
            .format(
            ptr='base_ptr = ' if has_parent else '',
            wrapper_name=self._wrapper_name(),
            id=collector_base_id - (1 if is_virtual else 0))

        for ctor in ctors:
            wrapper_return = '[ my_ptr, base_ptr ] = ' \
                if has_parent \
                else 'my_ptr = '

            methods_wrap += textwrap.indent(textwrap.dedent('''\
                elseif nargin == {len}{varargin}
                  {ptr}{wrapper}({num}{comma}{var_arg});
            ''').format(len=len(ctor.args.list()),
                        varargin=self._wrap_variable_arguments(
                            ctor.args, False),
                        ptr=wrapper_return,
                        wrapper=self._wrapper_name(),
                        num=self._update_wrapper_id(
                            (namespace_name, inst_class, 'constructor', ctor)),
                        comma='' if len(ctor.args.list()) == 0 else ', ',
                        var_arg=self._wrap_list_variable_arguments(ctor.args)),
                                            prefix='    ')

        base_obj = ''

        if has_parent:
            base_obj = '  obj = obj@{parent_name}(uint64(5139824614673773682), base_ptr);'.format(
                parent_name=parent_name)

        if base_obj:
            base_obj = '\n' + base_obj

        methods_wrap += textwrap.indent(textwrap.dedent('''\
              else
                error('Arguments do not match any overload of {class_name_doc} constructor');
              end{base_obj}
              obj.ptr_{class_name} = my_ptr;
            end\n
        ''').format(namespace=namespace_name,
                    d='' if namespace_name == '' else '.',
                    class_name_doc=self._format_class_name(inst_class,
                                                           separator="."),
                    class_name=self._format_class_name(inst_class,
                                                       separator=""),
                    base_obj=base_obj),
                                        prefix='  ')

        return methods_wrap

    def wrap_properties_block(self, class_name, inst_class):
        """Generate Matlab properties block of the class.

        E.g.
        ```
        properties
            ptr_gtsamISAM2Params = 0
            relinearizeSkip
        end
        ```

        Args:
            class_name: Class name with namespace to assign unique pointer.
            inst_class: The instantiated class whose properties we want to wrap.

        Returns:
            str: The `properties` block in a Matlab `classdef`.
        """
        # Get the property names and make into newline separated block
        class_pointer = "  ptr_{class_name} = 0".format(class_name=class_name)

        if len(inst_class.properties) > 0:
            properties = '\n' + "\n".join(
                ["  {}".format(p.name) for p in inst_class.properties])
        else:
            properties = ''

        properties = class_pointer + properties
        properties_block = textwrap.dedent('''\
            properties
            {properties}
            end
        ''').format(properties=properties)
        return properties_block

    def wrap_class_properties(self, namespace_name: str,
                              inst_class: InstantiatedClass):
        """Generate wrappers for the setters & getters of class properties.

        Args:
            inst_class: The instantiated class whose properties we wish to wrap.
        """
        properties = []
        for propty in inst_class.properties:
            # These are the setters and getters in the .m file
            function_name = namespace_name + inst_class.name + '_get_' + propty.name
            getter = """
            function varargout = get.{name}(this)
                {varargout} = {wrapper}({num}, this);
                this.{name} = {varargout};
            end
            """.format(name=propty.name,
                       varargout='varargout{1}',
                       wrapper=self._wrapper_name(),
                       num=self._update_wrapper_id(
                           (namespace_name, inst_class, propty.name, propty),
                           function_name=function_name))
            properties.append(getter)

            # Setter doesn't need varargin since it needs just one input.
            function_name = namespace_name + inst_class.name + '_set_' + propty.name
            setter = """
            function set.{name}(this, value)
                obj.{name} = value;
                {wrapper}({num}, this, value);
            end
            """.format(name=propty.name,
                       wrapper=self._wrapper_name(),
                       num=self._update_wrapper_id(
                           (namespace_name, inst_class, propty.name, propty),
                           function_name=function_name))
            properties.append(setter)

        return properties

    def wrap_class_deconstructor(self, namespace_name, inst_class):
        """Generate the delete function for the Matlab class."""
        class_name = inst_class.name

        methods_text = textwrap.indent(textwrap.dedent("""\
            function delete(obj)
              {wrapper}({num}, obj.ptr_{class_name});
            end\n
        """).format(num=self._update_wrapper_id(
            (namespace_name, inst_class, 'deconstructor', None)),
                    wrapper=self._wrapper_name(),
                    class_name="".join(inst_class.parent.full_namespaces()) +
                    class_name),
                                       prefix='  ')

        return methods_text

    def wrap_class_display(self):
        """Generate the display function for the Matlab class."""
        return textwrap.indent(textwrap.dedent("""\
            function display(obj), obj.print(''); end
            %DISPLAY Calls print on the object
            function disp(obj), obj.display; end
            %DISP Calls print on the object
        """),
                               prefix='  ')

    def _group_class_methods(self, methods):
        """Group overloaded methods together"""
        return self._group_methods(methods)

    @classmethod
    def _format_varargout(cls, return_type, return_type_formatted):
        """Determine format of return and varargout statements"""
        if cls._return_count(return_type) == 1:
            varargout = '' \
                if return_type_formatted == 'void' \
                else 'varargout{1} = '
        else:
            varargout = '[ varargout{1} varargout{2} ] = '

        return varargout

    def wrap_class_methods(self,
                           namespace_name,
                           inst_class,
                           methods,
                           serialize=(False, )):
        """Wrap the methods in the class.

        Args:
            namespace_name: the name of the class's namespace
            inst_class: the instantiated class whose methods to wrap
            methods: the methods to wrap in the order to wrap them
            serialize: mutable param storing if one of the methods is serialize
        """
        method_text = ''

        methods = self._group_class_methods(methods)

        # Convert to list so that it is mutable
        if isinstance(serialize, tuple):
            serialize = list(serialize)

        for method in methods:
            method_name = method[0].name
            if method_name in self.whitelist and method_name != 'serialize':
                continue
            if method_name in self.ignore_methods:
                continue

            if method_name == 'serialize':
                if self.use_boost_serialization:
                    serialize[0] = True
                    method_text += self.wrap_class_serialize_method(
                        namespace_name, inst_class)

            else:
                # Generate method code
                method_text += textwrap.indent(textwrap.dedent("""\
                    function varargout = {method_name}(this, varargin)
                    """).format(caps_name=method_name.upper(),
                                method_name=method_name),
                                               prefix='')

                for overload in method:
                    method_text += textwrap.indent(textwrap.dedent("""\
                    % {caps_name} usage: {method_name}(""").format(
                        caps_name=method_name.upper(),
                        method_name=method_name),
                                                   prefix='  ')

                    # Determine format of return and varargout statements
                    return_type_formatted = self._format_return_type(
                        overload.return_type,
                        include_namespace=True,
                        separator=".")
                    varargout = self._format_varargout(overload.return_type,
                                                       return_type_formatted)

                    check_statement = self._wrap_method_check_statement(
                        overload.args)
                    class_name = namespace_name + ('' if namespace_name == ''
                                                   else '.') + inst_class.name

                    end_statement = '' \
                        if check_statement == '' \
                        else textwrap.indent(textwrap.dedent("""\
                              return
                            end
                        """).format(
                        class_name=class_name,
                        method_name=overload.original.name), prefix='  ')

                    method_text += textwrap.dedent("""\
                        {method_args}) : returns {return_type}
                          % Doxygen can be found at https://gtsam.org/doxygen/
                          {check_statement}{spacing}{varargout}{wrapper}({num}, this, varargin{{:}});
                        {end_statement}""").format(
                        method_args=self._wrap_args(overload.args),
                        return_type=return_type_formatted,
                        num=self._update_wrapper_id(
                            (namespace_name, inst_class,
                             overload.original.name, overload)),
                        check_statement=check_statement,
                        spacing='' if check_statement == '' else '    ',
                        varargout=varargout,
                        wrapper=self._wrapper_name(),
                        end_statement=end_statement)

                final_statement = textwrap.indent(textwrap.dedent("""\
                    error('Arguments do not match any overload of function {class_name}.{method_name}');
                """.format(class_name=class_name, method_name=method_name)),
                                                  prefix='  ')
                method_text += final_statement + 'end\n\n'

        return method_text

    def wrap_static_methods(self, namespace_name, instantiated_class,
                            serialize):
        """
        Wrap the static methods in the class.
        """
        class_name = instantiated_class.name

        method_text = 'methods(Static = true)\n'
        static_methods = sorted(instantiated_class.static_methods,
                                key=lambda name: name.name)

        static_methods = self._group_class_methods(static_methods)

        for static_method in static_methods:
            format_name = list(static_method[0].name)
            format_name[0] = format_name[0]

            if static_method[0].name in self.ignore_methods:
                continue

            method_text += textwrap.indent(textwrap.dedent('''\
                    function varargout = {name}(varargin)
                    '''.format(name=''.join(format_name))),
                                           prefix="  ")

            for static_overload in static_method:
                check_statement = self._wrap_method_check_statement(
                    static_overload.args)

                end_statement = '' \
                    if check_statement == '' \
                    else textwrap.indent(textwrap.dedent("""
                              return
                            end
                            """), prefix='')
                method_text += textwrap.indent(textwrap.dedent('''\
                      % {name_caps} usage: {name_upper_case}({args}) : returns {return_type}
                      % Doxygen can be found at https://gtsam.org/doxygen/
                      {check_statement}{spacing}varargout{{1}} = {wrapper}({id}, varargin{{:}});{end_statement}
                      ''').format(
                    name=''.join(format_name),
                    name_caps=static_overload.name.upper(),
                    name_upper_case=static_overload.name,
                    args=self._wrap_args(static_overload.args),
                    return_type=self._format_return_type(
                        static_overload.return_type,
                        include_namespace=True,
                        separator="."),
                    length=len(static_overload.args.list()),
                    var_args_list=self._wrap_variable_arguments(
                        static_overload.args),
                    check_statement=check_statement,
                    spacing='' if check_statement == '' else '  ',
                    wrapper=self._wrapper_name(),
                    id=self._update_wrapper_id(
                        (namespace_name, instantiated_class,
                         static_overload.name, static_overload)),
                    class_name=instantiated_class.name,
                    end_statement=end_statement),
                                               prefix='    ')

            # If the arguments don't match any of the checks above,
            # throw an error with the class and method name.
            method_text += textwrap.indent(textwrap.dedent("""\
                    error('Arguments do not match any overload of function {class_name}.{method_name}');
                """.format(class_name=class_name,
                           method_name=static_overload.name)),
                                           prefix='    ')

            method_text += textwrap.indent(textwrap.dedent("""\
                                    end\n
                """),
                                           prefix="  ")

        if serialize and self.use_boost_serialization:
            method_text += WrapperTemplate.matlab_deserialize.format(
                class_name=namespace_name + '.' + instantiated_class.name,
                wrapper=self._wrapper_name(),
                id=self._update_wrapper_id(
                    (namespace_name, instantiated_class, 'string_deserialize',
                     'deserialize')))

        return method_text

    def wrap_instantiated_class(self,
                                instantiated_class,
                                namespace_name: str = ''):
        """Generate comments and code for given class.

        Args:
            instantiated_class: template_instantiator.InstantiatedClass
                instance storing the class to wrap
            namespace_name: the name of the namespace if there is one
        """
        file_name = self._clean_class_name(instantiated_class)
        namespace_file_name = namespace_name + file_name

        uninstantiated_name = "::".join(instantiated_class.namespaces()
                                        [1:]) + "::" + instantiated_class.name
        if uninstantiated_name in self.ignore_classes:
            return None

        # Class comment
        content_text = self.class_comment(instantiated_class)
        content_text += self.wrap_methods(instantiated_class.methods)

        # Class definition
        # if namespace_name:
        #     print("nsname: {}, file_name_: {}, filename: {}"
        #           .format(namespace_name,
        #                   self._clean_class_name(instantiated_class), file_name)
        #           , file=sys.stderr)
        content_text += 'classdef {class_name} < {parent}\n'.format(
            class_name=file_name,
            parent=str(self._qualified_name(
                instantiated_class.parent_class)).replace("::", "."))

        # Class properties
        content_text += '  ' + reduce(
            self._insert_spaces,
            self.wrap_properties_block(namespace_file_name,
                                       instantiated_class).splitlines()) + '\n'

        # Class constructor
        content_text += '  ' + reduce(
            self._insert_spaces,
            self.wrap_class_constructors(
                namespace_name,
                instantiated_class,
                instantiated_class.parent_class,
                instantiated_class.ctors,
                instantiated_class.is_virtual,
            ).splitlines()) + '\n'

        # Delete function
        content_text += '  ' + reduce(
            self._insert_spaces,
            self.wrap_class_deconstructor(
                namespace_name, instantiated_class).splitlines()) + '\n'

        # Display function
        content_text += '  ' + reduce(
            self._insert_spaces,
            self.wrap_class_display().splitlines()) + '\n'

        # Class methods
        serialize = [False]

        if len(instantiated_class.methods) != 0:
            methods = sorted(instantiated_class.methods,
                             key=lambda name: name.name)
            class_methods_wrapped = self.wrap_class_methods(
                namespace_name,
                instantiated_class,
                methods,
                serialize=serialize).splitlines()
            if len(class_methods_wrapped) > 0:
                content_text += '    ' + reduce(
                    lambda x, y: x + '\n' + ('' if y == '' else '    ') + y,
                    class_methods_wrapped) + '\n'

        # Class properties
        if len(instantiated_class.properties) != 0:
            property_accessors = self.wrap_class_properties(
                namespace_name, instantiated_class)
            content_text += textwrap.indent(textwrap.dedent(
                "".join(property_accessors)),
                                            prefix='    ')

        content_text += '  end'  # End the `methods` block

        # Static class methods
        content_text += '\n\n  ' + reduce(
            self._insert_spaces,
            self.wrap_static_methods(namespace_name, instantiated_class,
                                     serialize[0]).splitlines()) + '\n' + \
        '  end\n'

        # Close the classdef
        content_text += textwrap.dedent('''\
            end
        ''')

        return file_name + '.m', content_text

    def wrap_namespace(self, namespace):
        """Wrap a namespace by wrapping all of its components.

        Args:
            namespace: the interface_parser.namespace instance of the namespace
            parent: parent namespace
        """
        namespaces = namespace.full_namespaces()
        inner_namespace = namespace.name != ''
        wrapped = []

        cpp_filename = self._wrapper_name() + '.cpp'
        self.content.append((cpp_filename, self.wrapper_file_headers))

        current_scope = []
        namespace_scope = []

        for element in namespace.content:
            if isinstance(element, parser.Include):
                self.includes.append(element)

            elif isinstance(element, parser.Namespace):
                self.wrap_namespace(element)

            elif isinstance(element, instantiator.InstantiatedClass):
                self.add_class(element)

                if inner_namespace:
                    class_text = self.wrap_instantiated_class(
                        element, "".join(namespace.full_namespaces()))

                    if not class_text is None:
                        namespace_scope.append(("".join([
                            '+' + x + '/'
                            for x in namespace.full_namespaces()[1:]
                        ])[:-1], [(class_text[0], class_text[1])]))
                else:
                    class_text = self.wrap_instantiated_class(element)
                    current_scope.append((class_text[0], class_text[1]))

        self.content.extend(current_scope)

        if inner_namespace:
            self.content.append(namespace_scope)

        # Global functions
        all_funcs = [
            func for func in namespace.content
            if isinstance(func, parser.GlobalFunction)
        ]

        self.wrap_methods(all_funcs, True, global_ns=namespace)

        return wrapped

    def wrap_collector_function_shared_return(self,
                                              return_type_name,
                                              shared_obj,
                                              func_id,
                                              new_line=True):
        """Wrap the collector function which returns a shared pointer."""
        new_line = '\n' if new_line else ''

        return WrapperTemplate.collector_function_shared_return.format(
            name=self._format_type_name(return_type_name,
                                        include_namespace=False),
            shared_obj=shared_obj,
            id=func_id,
            new_line=new_line)

    def wrap_collector_function_return_types(self, return_type, func_id):
        """
        Wrap the return type of the collector function when a std::pair is returned.
        """
        return_type_text = '  out[' + str(func_id) + '] = '
        pair_value = 'first' if func_id == 0 else 'second'
        new_line = '\n' if func_id == 0 else ''

        if self.is_shared_ptr(return_type) or self.is_ptr(return_type) or \
            self.can_be_pointer(return_type):
            shared_obj = 'pairResult.' + pair_value

            if not (return_type.is_shared_ptr or return_type.is_ptr):
                shared_obj = 'std::make_shared<{name}>({shared_obj})' \
                    .format(name=self._format_type_name(return_type.typename),
                            shared_obj='pairResult.' + pair_value)

            if return_type.typename.name in self.ignore_namespace:
                return_type_text = self.wrap_collector_function_shared_return(
                    return_type.typename, shared_obj, func_id, func_id == 0)
            else:
                return_type_text += 'wrap_shared_ptr({0},"{1}", false);{new_line}' \
                    .format(shared_obj,
                            self._format_type_name(return_type.typename,
                                                   separator='.'),
                            new_line=new_line)
        else:
            return_type_text += 'wrap< {0} >(pairResult.{1});{2}'.format(
                self._format_type_name(return_type.typename, separator='.'),
                pair_value, new_line)

        return return_type_text

    def _collector_return(self, obj: str, ctype: parser.Type):
        """Helper method to get the final statement before the return in the collector function."""
        expanded = ''
        if self.is_shared_ptr(ctype) or self.is_ptr(ctype) or \
            self.can_be_pointer(ctype):
            sep_method_name = partial(self._format_type_name,
                                      ctype.typename,
                                      include_namespace=True)

            if ctype.typename.name in self.ignore_namespace:
                expanded += self.wrap_collector_function_shared_return(
                    ctype.typename, obj, 0, new_line=False)

            if ctype.is_shared_ptr or ctype.is_ptr:
                shared_obj = '{obj},"{method_name_sep}"'.format(
                    obj=obj, method_name_sep=sep_method_name('.'))
            else:
                method_name_sep_dot = sep_method_name('.')

                # Specialize for std::optional so we access the underlying member
                #TODO(Varun) How do we handle std::optional as a Mex type?
                if isinstance(ctype, parser.TemplatedType) and \
                    "std::optional" == str(ctype.typename)[:13]:
                    obj = f"*{obj}"
                    type_name = ctype.template_params[0].typename
                    method_name_sep_dot = ".".join(
                        type_name.namespaces) + f".{type_name.name}"


                shared_obj_template = 'std::make_shared<{method_name_sep_col}>({obj}),' \
                                        '"{method_name_sep_dot}"'
                shared_obj = shared_obj_template \
                    .format(method_name_sep_col=sep_method_name(),
                            method_name_sep_dot=method_name_sep_dot,
                            obj=obj)

            if ctype.typename.name not in self.ignore_namespace:
                expanded += textwrap.indent(
                    'out[0] = wrap_shared_ptr({0}, false);'.format(shared_obj),
                    prefix='  ')
        else:
            expanded += '  out[0] = wrap< {0} >({1});'.format(
                ctype.typename.name, obj)

        return expanded

    def wrap_collector_function_return(self, method):
        """
        Wrap the complete return type of the function.
        """
        expanded = ''

        params = self._wrapper_unwrap_arguments(method.args, arg_id=1)[0]

        return_1 = method.return_type.type1
        return_count = self._return_count(method.return_type)
        return_1_name = method.return_type.type1.typename.name
        obj_start = ''

        if isinstance(method, instantiator.InstantiatedMethod):
            method_name = method.to_cpp()
            obj_start = 'obj->'

            if method.instantiations:
                # method_name += '<{}>'.format(
                #     self._format_type_name(method.instantiations))
                method = method.to_cpp()

        elif isinstance(method, instantiator.InstantiatedStaticMethod):
            method_name = self._format_static_method(method, '::')
            method_name += method.original.name

        elif isinstance(method, parser.GlobalFunction):
            method_name = self._format_global_function(method, '::')
            method_name += method.name

        else:
            if isinstance(method.parent, instantiator.InstantiatedClass):
                method_name = method.parent.to_cpp() + "::"
            else:
                method_name = self._format_static_method(method, '::')
            method_name += method.name

        obj = '  ' if return_1_name == 'void' else ''
        obj += '{}{}({})'.format(obj_start, method_name, params)

        if return_1_name != 'void':
            if return_count == 1:
                expanded += self._collector_return(obj, return_1)

            elif return_count == 2:
                return_2 = method.return_type.type2

                expanded += '  auto pairResult = {};\n'.format(obj)
                expanded += self.wrap_collector_function_return_types(
                    return_1, 0)
                expanded += self.wrap_collector_function_return_types(
                    return_2, 1)
        else:
            expanded += obj + ';'

        return expanded

    def wrap_collector_property_return(self, class_property: parser.Variable):
        """Get the last collector function statement before return for a property."""
        property_name = class_property.name
        obj = 'obj->{}'.format(property_name)
        property_type = class_property.ctype

        return self._collector_return(obj, property_type)

    def wrap_collector_function_upcast_from_void(self, class_name, func_id,
                                                 cpp_name):
        """
        Add function to upcast type from void type.
        """
        return WrapperTemplate.collector_function_upcast_from_void.format(
            class_name=class_name, cpp_name=cpp_name, id=func_id)

    def generate_collector_function(self, func_id):
        """
        Generate the complete collector function that goes into the wrapper.cpp file.

        A collector function is the Mex function used to interact between
        the C++ object and the Matlab .m files.
        """
        collector_func = self.wrapper_map.get(func_id)

        if collector_func is None:
            return ''

        method_name = collector_func[3]

        collector_function = "void {}" \
            "(int nargout, mxArray *out[], int nargin, const mxArray *in[])\n".format(method_name)

        if isinstance(collector_func[1], instantiator.InstantiatedClass):
            body = '{\n'

            extra = collector_func[4]

            class_name = collector_func[0] + collector_func[1].name
            class_name_separated = collector_func[1].to_cpp()
            is_method = isinstance(extra, parser.Method)
            is_static_method = isinstance(extra, parser.StaticMethod)
            is_property = isinstance(extra, parser.Variable)

            if collector_func[2] == 'collectorInsertAndMakeBase':
                body += textwrap.indent(textwrap.dedent('''\
                    mexAtExit(&_deleteAllObjects);
                    typedef std::shared_ptr<{class_name_sep}> Shared;\n
                    Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
                    collector_{class_name}.insert(self);
                ''').format(class_name_sep=class_name_separated,
                            class_name=class_name),
                                        prefix='  ')

                if collector_func[1].parent_class:
                    body += textwrap.indent(textwrap.dedent('''
                        typedef std::shared_ptr<{}> SharedBase;
                        out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
                        *reinterpret_cast<SharedBase**>(mxGetData(out[0])) = new SharedBase(*self);
                    ''').format(collector_func[1].parent_class),
                                            prefix='  ')

            elif collector_func[2] == 'constructor':
                base = ''
                params, body_args = self._wrapper_unwrap_arguments(
                    extra.args, constructor=True)

                if collector_func[1].parent_class:
                    base += textwrap.indent(textwrap.dedent('''
                        typedef std::shared_ptr<{}> SharedBase;
                        out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
                        *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
                    ''').format(collector_func[1].parent_class),
                                            prefix='  ')

                body += textwrap.dedent('''\
                      mexAtExit(&_deleteAllObjects);
                      typedef std::shared_ptr<{class_name_sep}> Shared;\n
                    {body_args}  Shared *self = new Shared(new {class_name_sep}({params}));
                      collector_{class_name}.insert(self);
                      out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
                      *reinterpret_cast<Shared**> (mxGetData(out[0])) = self;
                    {base}''').format(class_name_sep=class_name_separated,
                                      body_args=body_args,
                                      params=params,
                                      class_name=class_name,
                                      base=base)

            elif collector_func[2] == 'deconstructor':
                body += textwrap.indent(textwrap.dedent('''\
                    typedef std::shared_ptr<{class_name_sep}> Shared;
                    checkArguments("delete_{class_name}",nargout,nargin,1);
                    Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
                    Collector_{class_name}::iterator item;
                    item = collector_{class_name}.find(self);
                    if(item != collector_{class_name}.end()) {{
                      collector_{class_name}.erase(item);
                    }}
                    delete self;
                ''').format(class_name_sep=class_name_separated,
                            class_name=class_name),
                                        prefix='  ')

            elif extra == 'serialize':
                if self.use_boost_serialization:
                    body += self.wrap_collector_function_serialize(
                        collector_func[1].name,
                        full_name=collector_func[1].to_cpp(),
                        namespace=collector_func[0])

            elif extra == 'deserialize':
                if self.use_boost_serialization:
                    body += self.wrap_collector_function_deserialize(
                        collector_func[1].name,
                        full_name=collector_func[1].to_cpp(),
                        namespace=collector_func[0])

            elif is_method or is_static_method:
                method_name = ''

                if is_static_method:
                    method_name = self._format_static_method(extra, '.')

                method_name += extra.name

                _, body_args = self._wrapper_unwrap_arguments(
                    extra.args, arg_id=1 if is_method else 0)
                return_body = self.wrap_collector_function_return(extra)

                shared_obj = ''

                if is_method:
                    shared_obj = '  auto obj = unwrap_shared_ptr<{class_name_sep}>' \
                            '(in[0], "ptr_{class_name}");\n'.format(
                                class_name_sep=class_name_separated,
                                class_name=class_name)

                body += '  checkArguments("{method_name}",nargout,nargin{min1},' \
                        '{num_args});\n' \
                        '{shared_obj}' \
                        '{body_args}' \
                        '{return_body}\n'.format(
                    min1='-1' if is_method else '',
                    shared_obj=shared_obj,
                    method_name=method_name,
                    num_args=len(extra.args.list()),
                    body_args=body_args,
                    return_body=return_body)

            elif is_property:
                shared_obj = '  auto obj = unwrap_shared_ptr<{class_name_sep}>' \
                            '(in[0], "ptr_{class_name}");\n'.format(
                                class_name_sep=class_name_separated,
                                class_name=class_name)

                # Unpack the property from mxArray
                property_type, unwrap = self._unwrap_argument(extra, arg_id=1)
                unpack_property = textwrap.indent(textwrap.dedent('''\
                    {arg_type} {name} = {unwrap}
                    '''.format(arg_type=property_type,
                               name=extra.name,
                               unwrap=unwrap)),
                                                  prefix='  ')

                # Getter
                if "_get_" in method_name:
                    return_body = self.wrap_collector_property_return(extra)

                    getter = '  checkArguments("{property_name}",nargout,nargin{min1},' \
                            '{num_args});\n' \
                            '{shared_obj}' \
                            '{return_body}\n'.format(
                        property_name=extra.name,
                        min1='-1',
                        num_args=0,
                        shared_obj=shared_obj,
                        return_body=return_body)

                    body += getter

                # Setter
                if "_set_" in method_name:
                    is_ptr_type = self.can_be_pointer(extra.ctype)
                    return_body = '  obj->{0} = {1}{0};'.format(
                        extra.name, '*' if is_ptr_type else '')

                    setter = '  checkArguments("{property_name}",nargout,nargin{min1},' \
                            '{num_args});\n' \
                            '{shared_obj}' \
                            '{unpack_property}' \
                            '{return_body}\n'.format(
                        property_name=extra.name,
                        min1='-1',
                        num_args=1,
                        shared_obj=shared_obj,
                        unpack_property=unpack_property,
                        return_body=return_body)

                    body += setter

            body += '}\n'

            if extra not in ['serialize', 'deserialize']:
                body += '\n'

            collector_function += body

        else:
            body = textwrap.dedent('''\
                {{
                  checkArguments("{function_name}",nargout,nargin,{len});
            ''').format(function_name=collector_func[1].name,
                        id=self.global_function_id,
                        len=len(collector_func[1].args.list()))

            body += self._wrapper_unwrap_arguments(collector_func[1].args)[1]
            body += self.wrap_collector_function_return(
                collector_func[1]) + '\n}\n'

            collector_function += body

            self.global_function_id += 1

        return collector_function

    def mex_function(self):
        """
        Generate the wrapped MEX function.
        """
        cases = ''
        next_case = None

        for wrapper_id in range(self.wrapper_id):
            id_val = self.wrapper_map.get(wrapper_id)
            set_next_case = False

            if id_val is None:
                id_val = self.wrapper_map.get(wrapper_id + 1)

                if id_val is None:
                    continue

                set_next_case = True

            cases += textwrap.indent(textwrap.dedent('''\
                case {}:
                  {}(nargout, out, nargin-1, in+1);
                  break;
                ''').format(wrapper_id, next_case if next_case else id_val[3]),
                                     prefix='    ')

            if set_next_case:
                next_case = '{}_upcastFromVoid_{}'.format(
                    id_val[1].name, wrapper_id + 1)
            else:
                next_case = None

        mex_function = WrapperTemplate.mex_function.format(
            module_name=self.module_name, cases=cases)

        return mex_function

    def get_class_name(self, cls):
        """Get the name of the class `cls` taking template instantiations into account."""
        if cls.instantiations:
            class_name_sep = cls.name
        else:
            class_name_sep = cls.to_cpp()

        class_name = self._format_class_name(cls)

        return class_name, class_name_sep

    def generate_preamble(self):
        """
        Generate the preamble of the wrapper file, which includes
        the Boost exports, typedefs for collectors, and
        the _deleteAllObjects and _RTTIRegister functions.
        """
        delete_objs = ''
        typedef_instances = []
        boost_class_export_guid = ''
        typedef_collectors = ''
        rtti_classes = ''

        for cls in self.classes:
            # Check if class is in ignore list.
            # If so, then skip
            uninstantiated_name = "::".join(cls.namespaces()[1:] + [cls.name])
            if uninstantiated_name in self.ignore_classes:
                continue

            class_name, class_name_sep = self.get_class_name(cls)

            # If a class has instantiations, then declare the typedef for each instance
            if cls.instantiations:
                cls_insts = ''
                for i, inst in enumerate(cls.instantiations):
                    if i != 0:
                        cls_insts += ', '

                    cls_insts += self._format_type_name(inst)

                typedef_instances.append('typedef {original_class_name} {class_name_sep};' \
                    .format(original_class_name=cls.to_cpp(),
                            class_name_sep=cls.name))

            # Get the Boost exports for serialization
            if self.use_boost_serialization and \
                cls.original.namespaces() and self._has_serialization(cls):
                boost_class_export_guid += 'BOOST_CLASS_EXPORT_GUID({}, "{}");\n'.format(
                    class_name_sep, class_name)

            # Typedef and declare the collector objects.
            typedef_collectors += WrapperTemplate.typdef_collectors.format(
                class_name_sep=class_name_sep, class_name=class_name)

            # Generate the _deleteAllObjects method
            delete_objs += WrapperTemplate.delete_obj.format(
                class_name=class_name)

            if cls.is_virtual:
                class_name, class_name_sep = self.get_class_name(cls)
                rtti_classes += '    types.insert(std::make_pair(typeid({}).name(), "{}"));\n' \
                    .format(class_name_sep, class_name)

        # Generate the typedef instances string
        typedef_instances = "\n".join(typedef_instances)

        # Generate the full deleteAllObjects function
        delete_all_objs = WrapperTemplate.delete_all_objects.format(
            delete_objs=delete_objs)

        # Generate the full RTTIRegister function
        rtti_register = WrapperTemplate.rtti_register.format(
            module_name=self.module_name, rtti_classes=rtti_classes)

        return typedef_instances, boost_class_export_guid, \
            typedef_collectors, delete_all_objs, rtti_register

    def generate_wrapper(self, namespace):
        """Generate the c++ wrapper."""
        assert namespace, "Namespace if empty"

        # Generate the header includes
        includes_list = sorted(self.includes,
                               key=lambda include: include.header)

        # If boost serialization is enabled, include serialization headers
        if self.use_boost_serialization:
            boost_headers = WrapperTemplate.boost_headers
        else:
            boost_headers = ""

        includes = textwrap.dedent("""\
            {wrapper_file_headers}
            {boost_headers}
            {includes_list}
        """).format(wrapper_file_headers=self.wrapper_file_headers.strip(),
                    boost_headers=boost_headers,
                    includes_list='\n'.join(map(str, includes_list)))

        preamble = self.generate_preamble()
        typedef_instances, boost_class_export_guid, \
            typedef_collectors, delete_all_objs, \
                rtti_register = preamble

        ptr_ctor_frag = ''
        set_next_case = False

        for idx in range(self.wrapper_id):
            id_val = self.wrapper_map.get(idx)
            queue_set_next_case = set_next_case

            set_next_case = False

            if id_val is None:
                id_val = self.wrapper_map.get(idx + 1)

                if id_val is None:
                    continue

                set_next_case = True

            ptr_ctor_frag += self.generate_collector_function(idx)

            if queue_set_next_case:
                ptr_ctor_frag += self.wrap_collector_function_upcast_from_void(
                    id_val[1].name, idx, id_val[1].to_cpp())

        wrapper_file = textwrap.dedent('''\
            {includes}
            {typedef_instances}
            {boost_class_export_guid}
            {typedefs_collectors}
            {delete_all_objs}
            {rtti_register}
            {pointer_constructor_fragment}{mex_function}''') \
            .format(includes=includes,
                    typedef_instances=typedef_instances,
                    boost_class_export_guid=boost_class_export_guid,
                    typedefs_collectors=typedef_collectors,
                    delete_all_objs=delete_all_objs,
                    rtti_register=rtti_register,
                    pointer_constructor_fragment=ptr_ctor_frag,
                    mex_function=self.mex_function())

        self.content.append((self._wrapper_name() + '.cpp', wrapper_file))

    def wrap_class_serialize_method(self, namespace_name, inst_class):
        """
        Wrap the serizalize method of the class.
        """
        class_name = inst_class.name
        wrapper_id = self._update_wrapper_id(
            (namespace_name, inst_class, 'string_serialize', 'serialize'))

        return WrapperTemplate.class_serialize_method.format(
            wrapper=self._wrapper_name(),
            wrapper_id=wrapper_id,
            class_name=namespace_name + '.' + class_name)

    def wrap_collector_function_serialize(self,
                                          class_name,
                                          full_name='',
                                          namespace=''):
        """
        Wrap the serizalize collector function.
        """
        return WrapperTemplate.collector_function_serialize.format(
            class_name=class_name, full_name=full_name, namespace=namespace)

    def wrap_collector_function_deserialize(self,
                                            class_name,
                                            full_name='',
                                            namespace=''):
        """
        Wrap the deserizalize collector function.
        """
        return WrapperTemplate.collector_function_deserialize.format(
            class_name=class_name, full_name=full_name, namespace=namespace)

    def generate_content(self, cc_content, path):
        """
        Generate files and folders from matlab wrapper content.

        Args:
            cc_content: The content to generate formatted as
                (file_name, file_content) or
                (folder_name, [(file_name, file_content)])
            path: The path to the files parent folder within the main folder
        """
        for c in cc_content:
            if isinstance(c, list):
                if len(c) == 0:
                    continue

                path_to_folder = osp.join(path, c[0][0])

                if not osp.isdir(path_to_folder):
                    try:
                        os.makedirs(path_to_folder, exist_ok=True)
                    except OSError:
                        pass

                for sub_content in c:
                    self.generate_content(sub_content[1], path_to_folder)

            elif isinstance(c[1], list):
                path_to_folder = osp.join(path, c[0])

                if not osp.isdir(path_to_folder):
                    try:
                        os.makedirs(path_to_folder, exist_ok=True)
                    except OSError:
                        pass
                for sub_content in c[1]:
                    path_to_file = osp.join(path_to_folder, sub_content[0])
                    with open(path_to_file, 'w') as f:
                        f.write(sub_content[1])
            else:
                path_to_file = osp.join(path, c[0])

                if not osp.isdir(path_to_file):
                    try:
                        os.mkdir(path)
                    except OSError:
                        pass

                with open(path_to_file, 'w') as f:
                    f.write(c[1])

    def wrap(self, files, path):
        """High level function to wrap the project."""
        content = ""
        modules = {}
        for file in files:
            with open(file, 'r') as f:
                content += f.read()

        # Parse the contents of the interface file
        parsed_result = parser.Module.parseString(content)

        # Instantiate the module
        module = instantiator.instantiate_namespace(parsed_result)

        if module.name in modules:
            modules[
                module.name].content[0].content += module.content[0].content
        else:
            modules[module.name] = module

        for module in modules.values():
            # Wrap the full namespace
            self.wrap_namespace(module)
            self.generate_wrapper(module)

            # Generate the corresponding .m and .cpp files
            self.generate_content(self.content, path)

        return self.content
