"""
Matlab wrapper
Author: Matthew Sklar
Date: March 2019
"""

import os
import argparse

import interface_parser as parser
import template_instantiator as instantiator

test_dir = "wrap/tests/"


class MatlabWrapper(object):
    # TODO: Define more types
    # TODO: May only need this for constructors
    data_type = {'int': 'numeric', 'double': 'double'}
    # TODO: Figure out what this number is
    val = 85

    def __init__(self, module, module_name, top_module_namespace='',
                 ignore_classes=[]):
        self.module = module
        self.module_name = module_name
        self.top_module_namespace = top_module_namespace
        self.ignore_classe = ignore_classes

        """ Generate the file starting comment for the given namespace """
        pass

    def _write_file(self, filename, content):
        """ Write the content to the specified file. """
        with open(test_dir + 'actual-matlab/' + filename, 'w') as f:
            f.write(content)

    def _mkdir(self, folder_name):
        """ Make a folder in the actual-matlab folder if it does not exist """
        folder_path = test_dir + 'actual-matlab/' + folder_name

        if not os.path.exists(folder_path):
            os.mkdir(folder_path)

    def _partial_match(self, namespace1, namespace2):
        """ Return if one array of namespaces is a subset of the other """
        for i in range(min(len(namespace1), len(namespace2))):
            if namespace1[i] != namespace2[i]:
                return False

        return True

    def _group_methods(self, methods):
        """ Group overloaded methods together """
        # Map method name to its index in the output
        method_map = {}
        method_out = []

        for method in methods:
            method_index = method_map.get(method.name)

            if method_index == None:
                method_map[method.name] = len(method_out)
                method_out.append([method])
            else:
                method_out[method_index].append(method)

        return method_out

    def _clean_class_name(self, instantiated_class):
        """ Reformated the C++ class name to fit Matlab defined naming standards """
        if len(instantiated_class.ctors) != 0:
            return instantiated_class.ctors[0].name

        # TODO: May have to group together same name templates if they don't have constructors
        # TODO: This would require another if statement if text before < is a duplicate method
        return instantiated_class.cpp_class()

    def class_comment(self, class_name, ctors, methods):
        comment = '%class {class_name}, see Doxygen page for details\n'\
            '%at http://research.cc.gatech.edu/borg/sites/edu.borg/html/index.html\n'\
            '%\n'\
            '%-------Constructors-------\n'.format(
                class_name=class_name)

        # Write constructors
        for ctor in ctors:
            comment += '%{ctor_name}('.format(ctor_name=ctor.name)

            for i, arg in enumerate(ctor.args.args_list):
                if i != 0:
                    comment += ', '

                comment += '{type} {name}'.format(
                    type=arg.ctype.typename.name, name=arg.name)

            comment += ')\n'

        comment += '%\n'\
            '%-------Methods-------\n'

        # Write methods
        for method in methods:
            comment += '%{name}('.format(name=method.name)

            for i, arg in enumerate(method.args.args_list):
                if i != 0:
                    comment += ', '

                comment += '{type} {name}'.format(
                    type=arg.ctype.typename.name, name=arg.name)

            comment += ') : returns {return_type}\n'.format(
                return_type=method.return_type.type1.typename.name)

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

    def wrap_global_method(self, method):
        """ Wrap the given global method """
        if type(method) != list:
            method = [method]

        m_method = method[0].name

        # Get all combinations of parameters
        param_list = [m.args for m in method]
        param_check = ''

        for i, p in enumerate(param_list):
            param_check += '      if' if i == 0 else '      elseif'
            param_check += ' length(varargin) == '

            if len(p.args_list) == 0:
                param_check += '0\n'
            else:
                param_check += str(len(p.args_list))

                for num, arg in enumerate(p.args_list):
                    param_check += " && isa(varargin{open}{num}{close},'{data_type}')".format(
                        open='{', num=num + 1, close='}', data_type=self.data_type[str(arg.ctype).strip()])

                param_check += '\n'

            # TODO: Figure out what number to put at val
            param_check += '        varargout{open}1{close} = {module_name}_wrapper({val}, varargin{open}:{close});\n'.format(
                open='{', close='}', module_name=self.module_name, val=self.val)
            self.val += 1

        param_check += '      else\n'\
            "        error('Arguments do not match any overload of function {func_name}');".format(
                func_name=m_method)

        return 'function varargout = {m_method}(varargin)\n'\
            '{statements}\n'\
            '      end\n'.format(
                m_method=m_method, statements=param_check)

    def wrap_methods(self, methods, globals=False):
        """ 
        Wrap a sequence of methods. Groups methods with the same names 
        together. If globals is True then output every method into its own file.
        """
        output = ''

        methods = self._group_methods(methods)

        for method in methods:
            if globals:
                method_text = self.wrap_global_method(method)
                self._write_file(method[0].name + '.m', method_text)
            else:
                method_text = self.wrap_method(method)
                output += ''

        return output

    def wrap_instantiated_class(self, instantiated_class):
        """ Wrap classes """
        file_name = self._clean_class_name(instantiated_class)

        # TODO: Create and output content
        content = self.class_comment(
            file_name, instantiated_class.ctors, instantiated_class.methods)
        content += self.wrap_methods(instantiated_class.methods)

        self._write_file(file_name + '.m', content)

    def wrap_namespace(self, namespace):
        # TODO: Add documentation
        wrapped = ''
        includes = ''

        test_output = ""

        namespaces = namespace.full_namespaces()

        if len(namespaces) < len(self.top_module_namespace):
            pass
        else:
            for element in namespace.content:
                if isinstance(element, parser.Include):
                    pass
                elif isinstance(element, parser.Namespace):
                    self._mkdir('+' + element.name)
                    self.wrap_namespace(element)
                elif isinstance(element, instantiator.InstantiatedClass):
                    self.wrap_instantiated_class(element)

        # Handle global functions
        all_funcs = [
            func for func in namespace.content
            if isinstance(func, parser.GlobalFunction)
        ]

        test_output += self.wrap_methods(all_funcs, True)

        return wrapped, includes

    def wrap(self):
        wrapped_namespace, includes = self.wrap_namespace(self.module)

        # TODO: Fix this
        return wrapped_namespace


if __name__ == "__main__":
    test_dir = "wrap/tests/"
    with open(test_dir + 'geometry.h', 'r') as f:
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
