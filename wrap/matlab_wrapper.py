import os
import argparse

import interface_parser as parser
import template_instantiator as instantiator


class MatlabWrapper(object):
    """ Wrap the given C++ code into Matlab.

    Keyword arguments:
    module -- the C++ module being wrapped
    module_name -- name of the C++ module being wrapped
    top_module_namespace -- C++ namespace for the top module (default '')
    ignore_classes -- A list of classes to ignore (default [])
    """

    # TODO: Define more types
    data_type = {'int': 'numeric', 'double': 'double'}

    # TODO: Figure out what this number is
    val = 85

    content = []

    def __init__(self, module, module_name, top_module_namespace='',
                 ignore_classes=[]):
        self.module = module
        self.module_name = module_name
        self.top_module_namespace = top_module_namespace
        self.ignore_classe = ignore_classes

    def _partial_match(self, namespace1, namespace2):
        """ Return if one array of namespaces is a subset of the other """
        for i in range(min(len(namespace1), len(namespace2))):
            if namespace1[i] != namespace2[i]:
                return False

        return True

    def _group_methods(self, methods):
        """ Group overloaded methods together """
        method_map = {}
        method_out = []

        for method in methods:
            method_index = method_map.get(method.name)

            if method_index is None:
                method_map[method.name] = len(method_out)
                method_out.append([method])
            else:
                method_out[method_index].append(method)

        return method_out

    def _clean_class_name(self, instantiated_class):
        """ Reformatted the C++ class name to fit Matlab defined naming
        standards """
        if len(instantiated_class.ctors) != 0:
            return instantiated_class.ctors[0].name

        # TODO: May have to group together same name templates if they don't
        # have constructors. This would require another if statement if text
        # before < is a duplicate method

        return instantiated_class.cpp_class()

    def _format_type_name(self, type_name):
        """ Format the given type name """
        formatted_type_name = ''

        for namespace in type_name.namespaces:
            formatted_type_name += namespace + '::'

        formatted_type_name += type_name.name

        return formatted_type_name

    def class_comment(self, class_name, ctors, methods):
        """ Generate comments for the given class in Matlab.

        Keyword arguments:
        class_name -- the name of the class being wrapped
        ctors -- a list of the constructors in the class
        methods -- a list of the methods in the class
        """
        comment = '%class {class_name}, see Doxygen page for details\n'\
            '%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/'\
            'index.html\n'.format(class_name=class_name)

        if len(ctors) != 0:
            comment += '%\n%-------Constructors-------\n'

        # Write constructors
        for ctor in ctors:
            comment += '%{ctor_name}('.format(ctor_name=ctor.name)

            for i, arg in enumerate(ctor.args.args_list):
                if i != 0:
                    comment += ', '

                comment += '{type} {name}'.format(
                    type=arg.ctype.typename.name, name=arg.name)

            comment += ')\n'

        if len(methods) != 0:
            comment += '%\n'\
                '%-------Methods-------\n'

        methods = sorted(methods, key=lambda name: name.name)

        # Write methods
        for method in methods:
            comment += '%{name}('.format(name=method.name)

            for i, arg in enumerate(method.args.args_list):
                if i != 0:
                    comment += ', '

                comment += '{type} {name}'.format(
                    type=arg.ctype.typename.name, name=arg.name)

            # TODO: Determine when to include namespace
            if method.return_type.type2 == '':
                return_type = self._format_type_name(
                    method.return_type.type1.typename)
            else:
                return_type = 'pair< {type1}, {type2} >'.format(
                    type1=self._format_type_name(
                        method.return_type.type1.typename),
                    type2=self._format_type_name(
                        method.return_type.type2.typename))

            comment += ') : returns {return_type}\n'.format(
                return_type=return_type)

        comment += '%\n'

        # TODO: Support more method types

        return comment

    def wrap_method(self, methods):
        """ Wrap methods in the body of a class """
        if type(methods) != list:
            methods = [methods]

        for method in methods:
            output = ''

        return ''

    def wrap_global_function(self, function):
        """ Wrap the given global function """
        if type(function) != list:
            function = [function]

        m_method = function[0].name

        # Get all combinations of parameters
        param_list = [m.args for m in function]
        param_check = ''

        for i, p in enumerate(param_list):
            param_check += '      if' if i == 0 else '      elseif'
            param_check += ' length(varargin) == '

            if len(p.args_list) == 0:
                param_check += '0\n'
            else:
                param_check += str(len(p.args_list))

                for num, arg in enumerate(p.args_list):
                    param_check += " && isa(varargin{open}{num}{close},"\
                        "'{data_type}')".format(
                            open='{', num=num + 1, close='}',
                            data_type=self.data_type[str(arg.ctype).strip()])

                param_check += '\n'

            # TODO: Figure out what number to put at val
            param_check += '        varargout{open}1{close} = {module_name}'\
                '_wrapper({val}, varargin{open}:{close});\n'.format(
                    open='{', close='}', module_name=self.module_name,
                    val=self.val)
            self.val += 1

        param_check += '      else\n'\
            "        error('Arguments do not match any overload of function "\
            "{func_name}');".format(func_name=m_method)

        return 'function varargout = {m_method}(varargin)\n'\
            '{statements}\n'\
            '      end\n'.format(m_method=m_method, statements=param_check)

    def wrap_methods(self, methods, globals=False):
        """
        Wrap a sequence of methods. Groups methods with the same names
        together. If globals is True then output every method into its own
        file.
        """
        output = ''

        methods = self._group_methods(methods)

        for method in methods:
            if globals:
                method_text = self.wrap_global_function(method)
                self.content.append((method[0].name + '.m', method_text))
            else:
                method_text = self.wrap_method(method)
                output += ''

        return output

    def wrap_instantiated_class(self, instantiated_class):
        """ Generate comments and code for given class """
        file_name = self._clean_class_name(instantiated_class)

        content_text = self.class_comment(
            file_name, instantiated_class.ctors, instantiated_class.methods)
        content_text += self.wrap_methods(instantiated_class.methods)

        # TODO: Generate file code

        return (file_name + '.m', content_text)

    def wrap_namespace(self, namespace):
        # TODO: Add documentation
        wrapped = []

        test_output = ""

        namespaces = namespace.full_namespaces()

        if len(namespaces) < len(self.top_module_namespace):
            pass
        else:
            for element in namespace.content:
                if isinstance(element, parser.Include):
                    pass
                elif isinstance(element, parser.Namespace):
                    self.wrap_namespace(element)
                elif isinstance(element, instantiator.InstantiatedClass):
                    class_text = self.wrap_instantiated_class(element)
                    self.content.append((class_text[0], class_text[1]))

        # Global functions
        all_funcs = [
            func for func in namespace.content
            if isinstance(func, parser.GlobalFunction)
        ]

        test_output += self.wrap_methods(all_funcs, True)

        return wrapped

    def wrap(self):
        wrapped_namespace = self.wrap_namespace(self.module)

        return self.content


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    arg_parser.add_argument(
        "--src", type=str, required=True, help="Input interface .h file.")
    arg_parser.add_argument(
        "--module_name",
        type=str,
        required=True,
        help="Name of the C++ class being wrapped.")
    arg_parser.add_argument(
        "--out",
        type=str,
        required=True,
        help="Name of the output folder.")
    arg_parser.add_argument(
        "--top_module_namespaces",
        type=str,
        default="",
        help="C++ namespace for the top module, e.g. `ns1::ns2::ns3`. "
        "Only the content within this namespace and its sub-namespaces "
        "will be wrapped. The content of this namespace will be available at "
        "the top module level, and its sub-namespaces' in the submodules.\n"
        "For example, `import <module_name>` gives you access to a Python "
        "`<module_name>.Class` of the corresponding C++ `ns1::ns2::ns3::Class`,"
        " and `from <module_name> import ns4` gives you access to a Python "
        "`ns4.Class` of the C++ `ns1::ns2::ns3::ns4::Class`. ")
    arg_parser.add_argument(
        "--ignore",
        nargs='*',
        type=str,
        help="A space-separated list of classes to ignore. "
        "Class names must include their full namespaces.")
    args = arg_parser.parse_args()

    top_module_namespaces = args.top_module_namespaces.split("::")
    if top_module_namespaces[0]:
        top_module_namespaces = [''] + top_module_namespaces

    with open(args.src, 'r') as f:
        content = f.read()

    module = parser.Module.parseString(content)

    instantiator.instantiate_namespace_inplace(module)

    wrapper = MatlabWrapper(
        module=module,
        module_name='geometry',
        top_module_namespace=[''],
        ignore_classes=['']
    )

    cc_content = wrapper.wrap()

    for c in cc_content:
        if type(c) == list:
            pass
        else:
            with open(args.out + '/' + c[0], 'w') as f:
                f.write(c[1])
