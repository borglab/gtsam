"""
Code to use the parsed results and convert it to a format
that Matlab's MEX compiler can use.
"""

# pylint: disable=too-many-lines, no-self-use, too-many-arguments, too-many-branches, too-many-statements

import os
import os.path as osp
import sys
import textwrap
from functools import partial, reduce
from typing import Dict, Iterable, List, Union

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator


class MatlabWrapper(object):
    """ Wrap the given C++ code into Matlab.

    Attributes
        module: the C++ module being wrapped
        module_name: name of the C++ module being wrapped
        top_module_namespace: C++ namespace for the top module (default '')
        ignore_classes: A list of classes to ignore (default [])
    """
    # Map the data type to its Matlab class.
    # Found in Argument.cpp in old wrapper
    data_type = {
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
    data_type_param = {
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
    # Methods that should not be wrapped directly
    whitelist = ['serializable', 'serialize']
    # Methods that should be ignored
    ignore_methods = ['pickle']
    # Datatypes that do not need to be checked in methods
    not_check_type: list = []
    # Data types that are primitive types
    not_ptr_type = ['int', 'double', 'bool', 'char', 'unsigned char', 'size_t']
    # Ignore the namespace for these datatypes
    ignore_namespace = ['Matrix', 'Vector', 'Point2', 'Point3']
    # The amount of times the wrapper has created a call to geometry_wrapper
    wrapper_id = 0
    # Map each wrapper id to what its collector function namespace, class, type, and string format
    wrapper_map: dict = {}
    # Set of all the includes in the namespace
    includes: Dict[parser.Include, int] = {}
    # Set of all classes in the namespace
    classes: List[Union[parser.Class, instantiator.InstantiatedClass]] = []
    classes_elems: Dict[Union[parser.Class, instantiator.InstantiatedClass], int] = {}
    # Id for ordering global functions in the wrapper
    global_function_id = 0
    # Files and their content
    content: List[str] = []

    # Ensure the template file is always picked up from the correct directory.
    dir_path = osp.dirname(osp.realpath(__file__))
    with open(osp.join(dir_path, "matlab_wrapper.tpl")) as f:
        wrapper_file_header = f.read()

    def __init__(self,
                 module,
                 module_name,
                 top_module_namespace='',
                 ignore_classes=()):
        self.module = module
        self.module_name = module_name
        self.top_module_namespace = top_module_namespace
        self.ignore_classes = ignore_classes
        self.verbose = False

    def _debug(self, message):
        if not self.verbose:
            return
        print(message, file=sys.stderr)

    def _add_include(self, include):
        self.includes[include] = 0

    def _add_class(self, instantiated_class):
        if self.classes_elems.get(instantiated_class) is None:
            self.classes_elems[instantiated_class] = 0
            self.classes.append(instantiated_class)

    def _update_wrapper_id(self, collector_function=None, id_diff=0):
        """Get and define wrapper ids.

        Generates the map of id -> collector function.

        Args:
            collector_function: tuple storing info about the wrapper function
                (namespace, class instance, function type, function name,
                extra)
            id_diff: constant to add to the id in the map

        Returns:
            the current wrapper id
        """
        if collector_function is not None:
            is_instantiated_class = isinstance(collector_function[1],
                                               instantiator.InstantiatedClass)

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

    def _is_shared_ptr(self, arg_type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        shared pointer in the wrapper.
        """
        return arg_type.is_shared_ptr or (
            arg_type.typename.name not in self.not_ptr_type
            and arg_type.typename.name not in self.ignore_namespace
            and arg_type.typename.name != 'string')

    def _is_ptr(self, arg_type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        raw pointer in the wrapper.
        """
        return arg_type.is_ptr or (
            arg_type.typename.name not in self.not_ptr_type
            and arg_type.typename.name not in self.ignore_namespace
            and arg_type.typename.name != 'string')

    def _is_ref(self, arg_type):
        """Determine if the interface_parser.Type should be treated as a
        reference in the wrapper.
        """
        return arg_type.typename.name not in self.ignore_namespace and \
               arg_type.typename.name not in self.not_ptr_type and \
               arg_type.is_ref

    def _group_methods(self, methods):
        """Group overloaded methods together"""
        method_map = {}
        method_out = []

        for method in methods:
            method_index = method_map.get(method.name)

            if method_index is None:
                method_map[method.name] = len(method_out)
                method_out.append([method])
            else:
                self._debug("[_group_methods] Merging {} with {}".format(
                    method_index, method.name))
                method_out[method_index].append(method)

        return method_out

    def _clean_class_name(self, instantiated_class):
        """Reformatted the C++ class name to fit Matlab defined naming
        standards
        """
        if len(instantiated_class.ctors) != 0:
            return instantiated_class.ctors[0].name

        return instantiated_class.name

    @classmethod
    def _format_type_name(cls,
                          type_name,
                          separator='::',
                          include_namespace=True,
                          constructor=False,
                          method=False):
        """
        Args:
            type_name: an interface_parser.Typename to reformat
            separator: the statement to add between namespaces and typename
            include_namespace: whether to include namespaces when reformatting
            constructor: if the typename will be in a constructor
            method: if the typename will be in a method

        Raises:
            constructor and method cannot both be true
        """
        if constructor and method:
            raise Exception(
                'Constructor and method parameters cannot both be True')

        formatted_type_name = ''
        name = type_name.name

        if include_namespace:
            for namespace in type_name.namespaces:
                if name not in cls.ignore_namespace and namespace != '':
                    formatted_type_name += namespace + separator

        #self._debug("formatted_ns: {}, ns: {}".format(formatted_type_name, type_name.namespaces))
        if constructor:
            formatted_type_name += cls.data_type.get(name) or name
        elif method:
            formatted_type_name += cls.data_type_param.get(name) or name
        else:
            formatted_type_name += name

        if separator == "::":  # C++
            templates = []
            for idx in range(len(type_name.instantiations)):
                template = '{}'.format(
                    cls._format_type_name(type_name.instantiations[idx],
                                          include_namespace=include_namespace,
                                          constructor=constructor,
                                          method=method))
                templates.append(template)

            if len(templates) > 0:  # If there are no templates
                formatted_type_name += '<{}>'.format(','.join(templates))

        else:
            for idx in range(len(type_name.instantiations)):
                formatted_type_name += '{}'.format(
                    cls._format_type_name(type_name.instantiations[idx],
                                          separator=separator,
                                          include_namespace=False,
                                          constructor=constructor,
                                          method=method))

        return formatted_type_name

    @classmethod
    def _format_return_type(cls,
                            return_type,
                            include_namespace=False,
                            separator="::"):
        """Format return_type.

        Args:
            return_type: an interface_parser.ReturnType to reformat
            include_namespace: whether to include namespaces when reformatting
        """
        return_wrap = ''

        if cls._return_count(return_type) == 1:
            return_wrap = cls._format_type_name(
                return_type.type1.typename,
                separator=separator,
                include_namespace=include_namespace)
        else:
            return_wrap = 'pair< {type1}, {type2} >'.format(
                type1=cls._format_type_name(
                    return_type.type1.typename,
                    separator=separator,
                    include_namespace=include_namespace),
                type2=cls._format_type_name(
                    return_type.type2.typename,
                    separator=separator,
                    include_namespace=include_namespace))

        return return_wrap

    def _format_class_name(self, instantiated_class, separator=''):
        """Format a template_instantiator.InstantiatedClass name."""
        if instantiated_class.parent == '':
            parent_full_ns = ['']
        else:
            parent_full_ns = instantiated_class.parent.full_namespaces()
        # class_name = instantiated_class.parent.name
        #
        # if class_name != '':
        #     class_name += separator
        #
        # class_name += instantiated_class.name
        parentname = "".join([separator + x
                              for x in parent_full_ns]) + separator

        class_name = parentname[2 * len(separator):]

        class_name += instantiated_class.name

        return class_name

    def _format_static_method(self, static_method, separator=''):
        """Example:

                gtsamPoint3.staticFunction
        """
        method = ''

        if isinstance(static_method, parser.StaticMethod):
            method += "".join([separator + x for x in static_method.parent.namespaces()]) + \
                      separator + static_method.parent.name + separator

        return method[2 * len(separator):]

    def _format_instance_method(self, instance_method, separator=''):
        """Example:

                gtsamPoint3.staticFunction
        """
        method = ''

        if isinstance(instance_method, instantiator.InstantiatedMethod):
            method_list = [
                separator + x
                for x in instance_method.parent.parent.full_namespaces()
            ]
            method += "".join(method_list) + separator

            method += instance_method.parent.name + separator
            method += instance_method.original.name
            method += "<" + instance_method.instantiations.to_cpp() + ">"

        return method[2 * len(separator):]

    def _format_global_method(self, static_method, separator=''):
        """Example:

                gtsamPoint3.staticFunction
        """
        method = ''

        if isinstance(static_method, parser.GlobalFunction):
            method += "".join([separator + x for x in static_method.parent.full_namespaces()]) + \
                      separator

        return method[2 * len(separator):]

    def _wrap_args(self, args):
        """Wrap an interface_parser.ArgumentList into a list of arguments.

        Returns:
            A string representation of the arguments. For example:
                'int x, double y'
        """
        arg_wrap = ''

        for i, arg in enumerate(args.args_list, 1):
            c_type = self._format_type_name(arg.ctype.typename,
                                            include_namespace=False)

            arg_wrap += '{c_type} {arg_name}{comma}'.format(
                c_type=c_type,
                arg_name=arg.name,
                comma='' if i == len(args.args_list) else ', ')

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

        for i, arg in enumerate(args.args_list, 1):
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
                    constructor=not wrap_datatypes)

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

        for i in range(1, len(args.args_list) + 1):
            if first:
                var_list_wrap += 'varargin{{{}}}'.format(i)
                first = False
            else:
                var_list_wrap += ', varargin{{{}}}'.format(i)

        return var_list_wrap

    def _wrap_method_check_statement(self, args):
        """
        Wrap the given arguments into either just a varargout call or a
        call in an if statement that checks if the parameters are accurate.
        """
        check_statement = ''
        arg_id = 1

        if check_statement == '':
            check_statement = \
                'if length(varargin) == {param_count}'.format(
                    param_count=len(args.args_list))

        for _, arg in enumerate(args.args_list):
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

    def _wrapper_unwrap_arguments(self, args, arg_id=0, constructor=False):
        """Format the interface_parser.Arguments.

        Examples:
            ((a), unsigned char a = unwrap< unsigned char >(in[1]);),
            ((a), Test& t = *unwrap_shared_ptr< Test >(in[1], "ptr_Test");),
            ((a), std::shared_ptr<Test> p1 = unwrap_shared_ptr< Test >(in[1], "ptr_Test");)
        """
        params = ''
        body_args = ''

        for arg in args.args_list:
            if params != '':
                params += ','

            if self._is_ref(arg.ctype):  # and not constructor:
                ctype_camel = self._format_type_name(arg.ctype.typename,
                                                     separator='')
                body_args += textwrap.indent(textwrap.dedent('''\
                   {ctype}& {name} = *unwrap_shared_ptr< {ctype} >(in[{id}], "ptr_{ctype_camel}");
                '''.format(ctype=self._format_type_name(arg.ctype.typename),
                           ctype_camel=ctype_camel,
                           name=arg.name,
                           id=arg_id)),
                                             prefix='  ')

            elif (self._is_shared_ptr(arg.ctype) or self._is_ptr(arg.ctype)) and \
                    arg.ctype.typename.name not in self.ignore_namespace:
                if arg.ctype.is_shared_ptr:
                    call_type = arg.ctype.is_shared_ptr
                else:
                    call_type = arg.ctype.is_ptr

                body_args += textwrap.indent(textwrap.dedent('''\
                    {std_boost}::shared_ptr<{ctype_sep}> {name} = unwrap_shared_ptr< {ctype_sep} >(in[{id}], "ptr_{ctype}");
                '''.format(std_boost='boost' if constructor else 'boost',
                           ctype_sep=self._format_type_name(
                               arg.ctype.typename),
                           ctype=self._format_type_name(arg.ctype.typename,
                                                        separator=''),
                           name=arg.name,
                           id=arg_id)),
                                             prefix='  ')
                if call_type == "":
                    params += "*"

            else:
                body_args += textwrap.indent(textwrap.dedent('''\
                    {ctype} {name} = unwrap< {ctype} >(in[{id}]);
                '''.format(ctype=arg.ctype.typename.name,
                           name=arg.name,
                           id=arg_id)),
                                             prefix='  ')

            params += arg.name

            arg_id += 1

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

    def generate_matlab_wrapper(self):
        """Generate the C++ file for the wrapper."""
        file_name = self._wrapper_name() + '.cpp'

        wrapper_file = self.wrapper_file_header

        return file_name, wrapper_file

    def wrap_method(self, methods):
        """Wrap methods in the body of a class."""
        if not isinstance(methods, list):
            methods = [methods]

        # for method in methods:
        #     output = ''

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
                self._debug("[wrap_methods] wrapping: {}..{}={}".format(
                    method[0].parent.name, method[0].name,
                    type(method[0].parent.name)))

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

            if len(overload.args.args_list) == 0:
                param_wrap += '0\n'
            else:
                param_wrap += str(len(overload.args.args_list)) \
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
        ''').format(func_name=function_name),
                                      prefix='      ')

        global_function = textwrap.indent(textwrap.dedent('''\
            function varargout = {m_method}(varargin)
            {statements}      end
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
            ''').format(len=len(ctor.args.args_list),
                        varargin=self._wrap_variable_arguments(
                            ctor.args, False),
                        ptr=wrapper_return,
                        wrapper=self._wrapper_name(),
                        num=self._update_wrapper_id(
                            (namespace_name, inst_class, 'constructor', ctor)),
                        comma='' if len(ctor.args.args_list) == 0 else ', ',
                        var_arg=self._wrap_list_variable_arguments(ctor.args)),
                                            prefix='    ')

        base_obj = ''

        if has_parent:
            self._debug("class: {} ns: {}".format(
                parent_name,
                self._format_class_name(inst_class.parent, separator=".")))

        if has_parent:
            base_obj = '  obj = obj@{parent_name}(uint64(5139824614673773682), base_ptr);'.format(
                parent_name=parent_name)

        if base_obj:
            base_obj = '\n' + base_obj

        self._debug("class: {}, name: {}".format(
            inst_class.name, self._format_class_name(inst_class,
                                                     separator=".")))
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

    def wrap_class_properties(self, class_name):
        """Generate properties of class."""
        return textwrap.dedent('''\
            properties
              ptr_{} = 0
            end
        ''').format(class_name)

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
        method_map = {}
        method_out = []

        for method in methods:
            method_index = method_map.get(method.name)

            if method_index is None:
                method_map[method.name] = len(method_out)
                method_out.append([method])
            else:
                # print("[_group_methods] Merging {} with {}".format(method_index, method.name))
                method_out[method_index].append(method)

        return method_out

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
                           serialize=(False,)):
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
            format_name[0] = format_name[0].upper()

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
                    length=len(static_overload.args.args_list),
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

            #TODO Figure out what is static_overload doing here.
            method_text += textwrap.indent(textwrap.dedent("""\
                    error('Arguments do not match any overload of function {class_name}.{method_name}');
                """.format(class_name=class_name,
                           method_name=static_overload.name)),
                                           prefix='    ')

            method_text += textwrap.indent(textwrap.dedent("""\
                                    end\n
                """), prefix="  ")

        if serialize:
            method_text += textwrap.indent(textwrap.dedent("""\
                function varargout = string_deserialize(varargin)
                  % STRING_DESERIALIZE usage: string_deserialize() : returns {class_name}
                  % Doxygen can be found at https://gtsam.org/doxygen/
                  if length(varargin) == 1
                    varargout{{1}} = {wrapper}({id}, varargin{{:}});
                  else
                    error('Arguments do not match any overload of function {class_name}.string_deserialize');
                  end
                end\n
                function obj = loadobj(sobj)
                  % LOADOBJ Saves the object to a matlab-readable format
                  obj = {class_name}.string_deserialize(sobj);
                end
            """).format(
                class_name=namespace_name + '.' + instantiated_class.name,
                wrapper=self._wrapper_name(),
                id=self._update_wrapper_id(
                    (namespace_name, instantiated_class, 'string_deserialize',
                     'deserialize'))),
                                           prefix='  ')

        return method_text

    def wrap_instantiated_class(self, instantiated_class, namespace_name=''):
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
            self.wrap_class_properties(
                namespace_file_name).splitlines()) + '\n'

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

        # Static class methods
        content_text += '  end\n\n  ' + reduce(
            self._insert_spaces,
            self.wrap_static_methods(namespace_name, instantiated_class,
                                     serialize[0]).splitlines()) + '\n'

        content_text += textwrap.dedent('''\
              end
            end
        ''')

        return file_name + '.m', content_text

    def wrap_namespace(self, namespace, parent=()):
        """Wrap a namespace by wrapping all of its components.

        Args:
            namespace: the interface_parser.namespace instance of the namespace
            parent: parent namespace
        """
        test_output = ''
        namespaces = namespace.full_namespaces()
        inner_namespace = namespace.name != ''
        wrapped = []
        self._debug("wrapping ns: {}, parent: {}".format(
            namespace.full_namespaces(), parent))

        matlab_wrapper = self.generate_matlab_wrapper()
        self.content.append((matlab_wrapper[0], matlab_wrapper[1]))

        current_scope = []
        namespace_scope = []

        for element in namespace.content:
            if isinstance(element, parser.Include):
                self._add_include(element)
            elif isinstance(element, parser.Namespace):
                self.wrap_namespace(element, namespaces)
            elif isinstance(element, instantiator.InstantiatedClass):
                self._add_class(element)

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

        test_output += self.wrap_methods(all_funcs, True, global_ns=namespace)

        return wrapped

    def wrap_collector_function_shared_return(self,
                                              return_type_name,
                                              shared_obj,
                                              func_id,
                                              new_line=True):
        """Wrap the collector function which returns a shared pointer."""
        new_line = '\n' if new_line else ''

        return textwrap.indent(textwrap.dedent('''\
            {{
            boost::shared_ptr<{name}> shared({shared_obj});
            out[{id}] = wrap_shared_ptr(shared,"{name}");
            }}{new_line}''').format(name=self._format_type_name(
            return_type_name, include_namespace=False),
                                    shared_obj=shared_obj,
                                    id=func_id,
                                    new_line=new_line),
                               prefix='  ')

    def wrap_collector_function_return_types(self, return_type, func_id):
        """
        Wrap the return type of the collector function.
        """
        return_type_text = '  out[' + str(func_id) + '] = '
        pair_value = 'first' if func_id == 0 else 'second'
        new_line = '\n' if func_id == 0 else ''

        if self._is_shared_ptr(return_type) or self._is_ptr(return_type):
            shared_obj = 'pairResult.' + pair_value

            if not (return_type.is_shared_ptr or return_type.is_ptr):
                shared_obj = 'boost::make_shared<{name}>({shared_obj})' \
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
            # method_name = method.original.name
            method_name = method.to_cpp()
            obj_start = 'obj->'

            if method.instantiations:
                # method_name += '<{}>'.format(
                #     self._format_type_name(method.instantiations))
                # method_name = self._format_instance_method(method, '::')
                method = method.to_cpp()

        elif isinstance(method, parser.GlobalFunction):
            method_name = self._format_global_method(method, '::')
            method_name += method.name

        else:
            if isinstance(method.parent, instantiator.InstantiatedClass):
                method_name = method.parent.cpp_class() + "::"
            else:
                method_name = self._format_static_method(method, '::')
            method_name += method.name

        if "MeasureRange" in method_name:
            self._debug("method: {}, method: {}, inst: {}".format(
                method_name, method.name, method.parent.cpp_class()))

        obj = '  ' if return_1_name == 'void' else ''
        obj += '{}{}({})'.format(obj_start, method_name, params)

        if return_1_name != 'void':
            if return_count == 1:
                if self._is_shared_ptr(return_1) or self._is_ptr(return_1):
                    sep_method_name = partial(self._format_type_name,
                                              return_1.typename,
                                              include_namespace=True)

                    if return_1.typename.name in self.ignore_namespace:
                        expanded += self.wrap_collector_function_shared_return(
                            return_1.typename, obj, 0, new_line=False)

                    if return_1.is_shared_ptr or return_1.is_ptr:
                        shared_obj = '{obj},"{method_name_sep}"'.format(
                            obj=obj, method_name_sep=sep_method_name('.'))
                    else:
                        self._debug("Non-PTR: {}, {}".format(
                            return_1, type(return_1)))
                        self._debug("Inner type is: {}, {}".format(
                            return_1.typename.name, sep_method_name('.')))
                        self._debug("Inner type instantiations: {}".format(
                            return_1.typename.instantiations))
                        method_name_sep_dot = sep_method_name('.')
                        shared_obj_template = 'boost::make_shared<{method_name_sep_col}>({obj}),' \
                                              '"{method_name_sep_dot}"'
                        shared_obj = shared_obj_template \
                            .format(method_name_sep_col=sep_method_name(),
                                    method_name_sep_dot=method_name_sep_dot,
                                    obj=obj)

                    if return_1.typename.name not in self.ignore_namespace:
                        expanded += textwrap.indent(
                            'out[0] = wrap_shared_ptr({}, false);'.format(
                                shared_obj),
                            prefix='  ')
                else:
                    expanded += '  out[0] = wrap< {} >({});'.format(
                        return_1.typename.name, obj)
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

    def wrap_collector_function_upcast_from_void(self, class_name, func_id,
                                                 cpp_name):
        """
        Add function to upcast type from void type.
        """
        return textwrap.dedent('''\
            void {class_name}_upcastFromVoid_{id}(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {{
              mexAtExit(&_deleteAllObjects);
              typedef boost::shared_ptr<{cpp_name}> Shared;
              boost::shared_ptr<void> *asVoid = *reinterpret_cast<boost::shared_ptr<void>**> (mxGetData(in[0]));
              out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
              Shared *self = new Shared(boost::static_pointer_cast<{cpp_name}>(*asVoid));
              *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
            }}\n
        ''').format(class_name=class_name, cpp_name=cpp_name, id=func_id)

    def generate_collector_function(self, func_id):
        """
        Generate the complete collector function.
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
            class_name_separated = collector_func[1].cpp_class()
            is_method = isinstance(extra, parser.Method)
            is_static_method = isinstance(extra, parser.StaticMethod)

            if collector_func[2] == 'collectorInsertAndMakeBase':
                body += textwrap.indent(textwrap.dedent('''\
                    mexAtExit(&_deleteAllObjects);
                    typedef boost::shared_ptr<{class_name_sep}> Shared;\n
                    Shared *self = *reinterpret_cast<Shared**> (mxGetData(in[0]));
                    collector_{class_name}.insert(self);
                ''').format(class_name_sep=class_name_separated,
                            class_name=class_name),
                                        prefix='  ')

                if collector_func[1].parent_class:
                    body += textwrap.indent(textwrap.dedent('''
                        typedef boost::shared_ptr<{}> SharedBase;
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
                        typedef boost::shared_ptr<{}> SharedBase;
                        out[1] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
                        *reinterpret_cast<SharedBase**>(mxGetData(out[1])) = new SharedBase(*self);
                    ''').format(collector_func[1].parent_class),
                                            prefix='  ')

                body += textwrap.dedent('''\
                      mexAtExit(&_deleteAllObjects);
                      typedef boost::shared_ptr<{class_name_sep}> Shared;\n
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
                    typedef boost::shared_ptr<{class_name_sep}> Shared;
                    checkArguments("delete_{class_name}",nargout,nargin,1);
                    Shared *self = *reinterpret_cast<Shared**>(mxGetData(in[0]));
                    Collector_{class_name}::iterator item;
                    item = collector_{class_name}.find(self);
                    if(item != collector_{class_name}.end()) {{
                      delete self;
                      collector_{class_name}.erase(item);
                    }}
                ''').format(class_name_sep=class_name_separated,
                            class_name=class_name),
                                        prefix='  ')
            elif extra == 'serialize':
                body += self.wrap_collector_function_serialize(
                    collector_func[1].name,
                    full_name=collector_func[1].cpp_class(),
                    namespace=collector_func[0])
            elif extra == 'deserialize':
                body += self.wrap_collector_function_deserialize(
                    collector_func[1].name,
                    full_name=collector_func[1].cpp_class(),
                    namespace=collector_func[0])
            elif is_method or is_static_method:
                method_name = ''

                if is_static_method:
                    method_name = self._format_static_method(extra) + '.'

                method_name += extra.name

                # return_type = extra.return_type
                # return_count = self._return_count(return_type)

                return_body = self.wrap_collector_function_return(extra)
                params, body_args = self._wrapper_unwrap_arguments(
                    extra.args, arg_id=1 if is_method else 0)

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
                    num_args=len(extra.args.args_list),
                    body_args=body_args,
                    return_body=return_body)

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
                        len=len(collector_func[1].args.args_list))

            body += self._wrapper_unwrap_arguments(collector_func[1].args)[1]
            body += self.wrap_collector_function_return(collector_func[1]) + '\n}\n'

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

        mex_function = textwrap.dedent('''
            void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
            {{
              mstream mout;
              std::streambuf *outbuf = std::cout.rdbuf(&mout);\n
              _{module_name}_RTTIRegister();\n
              int id = unwrap<int>(in[0]);\n
              try {{
                switch(id) {{
            {cases}    }}
              }} catch(const std::exception& e) {{
                mexErrMsgTxt(("Exception from gtsam:\\n" + std::string(e.what()) + "\\n").c_str());
              }}\n
              std::cout.rdbuf(outbuf);
            }}
        ''').format(module_name=self.module_name, cases=cases)

        return mex_function

    def generate_wrapper(self, namespace):
        """Generate the c++ wrapper."""
        # Includes
        wrapper_file = self.wrapper_file_header + textwrap.dedent("""
            #include <boost/archive/text_iarchive.hpp>
            #include <boost/archive/text_oarchive.hpp>
            #include <boost/serialization/export.hpp>\n
        """)

        assert namespace

        includes_list = sorted(list(self.includes.keys()),
                               key=lambda include: include.header)

        # Check the number of includes.
        # If no includes, do nothing, if 1 then just append newline.
        # if more than one, concatenate them with newlines.
        if len(includes_list) == 0:
            pass
        elif len(includes_list) == 1:
            wrapper_file += (str(includes_list[0]) + '\n')
        else:
            wrapper_file += reduce(lambda x, y: str(x) + '\n' + str(y),
                                   includes_list)
        wrapper_file += '\n'

        typedef_instances = '\n'
        typedef_collectors = ''
        boost_class_export_guid = ''
        delete_objs = textwrap.dedent('''\
            void _deleteAllObjects()
            {
              mstream mout;
              std::streambuf *outbuf = std::cout.rdbuf(&mout);\n
              bool anyDeleted = false;
        ''')
        rtti_reg_start = textwrap.dedent('''\
            void _{module_name}_RTTIRegister() {{
              const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_{module_name}_rttiRegistry_created");
              if(!alreadyCreated) {{
                std::map<std::string, std::string> types;
        ''').format(module_name=self.module_name)
        rtti_reg_mid = ''
        rtti_reg_end = textwrap.indent(
            textwrap.dedent('''
                mxArray *registry = mexGetVariable("global", "gtsamwrap_rttiRegistry");
                if(!registry)
                  registry = mxCreateStructMatrix(1, 1, 0, NULL);
                typedef std::pair<std::string, std::string> StringPair;
                for(const StringPair& rtti_matlab: types) {
                  int fieldId = mxAddField(registry, rtti_matlab.first.c_str());
                  if(fieldId < 0)
                    mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
                  mxArray *matlabName = mxCreateString(rtti_matlab.second.c_str());
                  mxSetFieldByNumber(registry, 0, fieldId, matlabName);
                }
                if(mexPutVariable("global", "gtsamwrap_rttiRegistry", registry) != 0)
                  mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
                mxDestroyArray(registry);
        '''),
            prefix='    ') + '    \n' + textwrap.dedent('''\
                mxArray *newAlreadyCreated = mxCreateNumericMatrix(0, 0, mxINT8_CLASS, mxREAL);
                if(mexPutVariable("global", "gtsam_geometry_rttiRegistry_created", newAlreadyCreated) != 0)
                  mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
                mxDestroyArray(newAlreadyCreated);
              }
            }
        ''')
        ptr_ctor_frag = ''

        for cls in self.classes:
            uninstantiated_name = "::".join(
                cls.namespaces()[1:]) + "::" + cls.name
            self._debug("Cls: {} -> {}".format(cls.name, uninstantiated_name))

            if uninstantiated_name in self.ignore_classes:
                self._debug("Ignoring: {} -> {}".format(
                    cls.name, uninstantiated_name))
                continue

            def _has_serialization(cls):
                for m in cls.methods:
                    if m.name in self.whitelist:
                        return True
                return False

            if cls.instantiations:
                cls_insts = ''

                for i, inst in enumerate(cls.instantiations):
                    if i != 0:
                        cls_insts += ', '

                    cls_insts += self._format_type_name(inst)

                typedef_instances += 'typedef {original_class_name} {class_name_sep};\n' \
                    .format(original_class_name=cls.cpp_class(),
                            class_name_sep=cls.name)

                class_name_sep = cls.name
                class_name = self._format_class_name(cls)

                if len(cls.original.namespaces()) > 1 and _has_serialization(
                        cls):
                    boost_class_export_guid += 'BOOST_CLASS_EXPORT_GUID({}, "{}");\n'.format(
                        class_name_sep, class_name)
            else:
                class_name_sep = cls.cpp_class()
                class_name = self._format_class_name(cls)

                if len(cls.original.namespaces()) > 1 and _has_serialization(
                        cls):
                    boost_class_export_guid += 'BOOST_CLASS_EXPORT_GUID({}, "{}");\n'.format(
                        class_name_sep, class_name)

            typedef_collectors += textwrap.dedent('''\
                typedef std::set<boost::shared_ptr<{class_name_sep}>*> Collector_{class_name};
                static Collector_{class_name} collector_{class_name};
            ''').format(class_name_sep=class_name_sep, class_name=class_name)
            delete_objs += textwrap.indent(textwrap.dedent('''\
                {{ for(Collector_{class_name}::iterator iter = collector_{class_name}.begin();
                    iter != collector_{class_name}.end(); ) {{
                  delete *iter;
                  collector_{class_name}.erase(iter++);
                  anyDeleted = true;
                }} }}
            ''').format(class_name=class_name),
                                           prefix='  ')

            if cls.is_virtual:
                rtti_reg_mid += '    types.insert(std::make_pair(typeid({}).name(), "{}"));\n' \
                    .format(class_name_sep, class_name)

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
                    id_val[1].name, idx, id_val[1].cpp_class())

        wrapper_file += textwrap.dedent('''\
            {typedef_instances}
            {boost_class_export_guid}
            {typedefs_collectors}
            {delete_objs}  if(anyDeleted)
                cout <<
                  "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\\n"
                  "calling destructors, call \'clear all\' again if you plan to now recompile a wrap\\n"
                  "module, so that your recompiled module is used instead of the old one." << endl;
              std::cout.rdbuf(outbuf);
            }}\n
            {rtti_register}
            {pointer_constructor_fragment}{mex_function}''') \
            .format(typedef_instances=typedef_instances,
                    boost_class_export_guid=boost_class_export_guid,
                    typedefs_collectors=typedef_collectors,
                    delete_objs=delete_objs,
                    rtti_register=rtti_reg_start + rtti_reg_mid + rtti_reg_end,
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

        return textwrap.dedent('''\
            function varargout = string_serialize(this, varargin)
              % STRING_SERIALIZE usage: string_serialize() : returns string
              % Doxygen can be found at https://gtsam.org/doxygen/
              if length(varargin) == 0
                varargout{{1}} = {wrapper}({wrapper_id}, this, varargin{{:}});
              else
                error('Arguments do not match any overload of function {class_name}.string_serialize');
              end
            end\n
            function sobj = saveobj(obj)
              % SAVEOBJ Saves the object to a matlab-readable format
              sobj = obj.string_serialize();
            end
        ''').format(wrapper=self._wrapper_name(),
                    wrapper_id=wrapper_id,
                    class_name=namespace_name + '.' + class_name)

    def wrap_collector_function_serialize(self,
                                          class_name,
                                          full_name='',
                                          namespace=''):
        """
        Wrap the serizalize collector function.
        """
        return textwrap.indent(textwrap.dedent("""\
            typedef boost::shared_ptr<{full_name}> Shared;
            checkArguments("string_serialize",nargout,nargin-1,0);
            Shared obj = unwrap_shared_ptr<{full_name}>(in[0], "ptr_{namespace}{class_name}");
            ostringstream out_archive_stream;
            boost::archive::text_oarchive out_archive(out_archive_stream);
            out_archive << *obj;
            out[0] = wrap< string >(out_archive_stream.str());
        """).format(class_name=class_name,
                    full_name=full_name,
                    namespace=namespace),
                               prefix='  ')

    def wrap_collector_function_deserialize(self,
                                            class_name,
                                            full_name='',
                                            namespace=''):
        """
        Wrap the deserizalize collector function.
        """
        return textwrap.indent(textwrap.dedent("""\
            typedef boost::shared_ptr<{full_name}> Shared;
            checkArguments("{namespace}{class_name}.string_deserialize",nargout,nargin,1);
            string serialized = unwrap< string >(in[0]);
            istringstream in_archive_stream(serialized);
            boost::archive::text_iarchive in_archive(in_archive_stream);
            Shared output(new {full_name}());
            in_archive >> *output;
            out[0] = wrap_shared_ptr(output,"{namespace}.{class_name}", false);
        """).format(class_name=class_name,
                    full_name=full_name,
                    namespace=namespace),
                               prefix='  ')

    def wrap(self):
        """High level function to wrap the project."""
        self.wrap_namespace(self.module)
        self.generate_wrapper(self.module)

        return self.content


def generate_content(cc_content, path, verbose=False):
    """
    Generate files and folders from matlab wrapper content.

    Args:
        cc_content: The content to generate formatted as
            (file_name, file_content) or
            (folder_name, [(file_name, file_content)])
        path: The path to the files parent folder within the main folder
    """
    def _debug(message):
        if not verbose:
            return
        print(message, file=sys.stderr)

    for c in cc_content:
        if isinstance(c, list):
            if len(c) == 0:
                continue
            _debug("c object: {}".format(c[0][0]))
            path_to_folder = osp.join(path, c[0][0])

            if not os.path.isdir(path_to_folder):
                try:
                    os.makedirs(path_to_folder, exist_ok=True)
                except OSError:
                    pass

            for sub_content in c:
                _debug("sub object: {}".format(sub_content[1][0][0]))
                generate_content(sub_content[1], path_to_folder)

        elif isinstance(c[1], list):
            path_to_folder = osp.join(path, c[0])

            _debug("[generate_content_global]: {}".format(path_to_folder))
            if not os.path.isdir(path_to_folder):
                try:
                    os.makedirs(path_to_folder, exist_ok=True)
                except OSError:
                    pass
            for sub_content in c[1]:
                path_to_file = osp.join(path_to_folder, sub_content[0])
                _debug("[generate_global_method]: {}".format(path_to_file))
                with open(path_to_file, 'w') as f:
                    f.write(sub_content[1])
        else:
            path_to_file = osp.join(path, c[0])

            _debug("[generate_content]: {}".format(path_to_file))
            if not os.path.isdir(path_to_file):
                try:
                    os.mkdir(path)
                except OSError:
                    pass

            with open(path_to_file, 'w') as f:
                f.write(c[1])
