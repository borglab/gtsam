"""Mixins for reducing the amount of boilerplate in the main wrapper class."""

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator


class CheckMixin:
    """Mixin to provide various checks."""
    # Data types that are primitive types
    not_ptr_type = ['int', 'double', 'bool', 'char', 'unsigned char', 'size_t']
    # Ignore the namespace for these datatypes
    ignore_namespace = ['Matrix', 'Vector', 'Point2', 'Point3']
    # Methods that should be ignored
    ignore_methods = ['pickle']
    # Methods that should not be wrapped directly
    whitelist = ['serializable', 'serialize']
    # Datatypes that do not need to be checked in methods
    not_check_type: list = []

    def _has_serialization(self, cls):
        for m in cls.methods:
            if m.name in self.whitelist:
                return True
        return False

    def is_shared_ptr(self, arg_type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        shared pointer in the wrapper.
        """
        return arg_type.is_shared_ptr or (
            arg_type.typename.name not in self.not_ptr_type
            and arg_type.typename.name not in self.ignore_namespace
            and arg_type.typename.name != 'string')

    def is_ptr(self, arg_type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        raw pointer in the wrapper.
        """
        return arg_type.is_ptr or (
            arg_type.typename.name not in self.not_ptr_type
            and arg_type.typename.name not in self.ignore_namespace
            and arg_type.typename.name != 'string')

    def is_ref(self, arg_type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        reference in the wrapper.
        """
        return arg_type.typename.name not in self.ignore_namespace and \
               arg_type.typename.name not in self.not_ptr_type and \
               arg_type.is_ref


class FormatMixin:
    """Mixin to provide formatting utilities."""
    def _clean_class_name(self, instantiated_class):
        """Reformatted the C++ class name to fit Matlab defined naming
        standards
        """
        if len(instantiated_class.ctors) != 0:
            return instantiated_class.ctors[0].name

        return instantiated_class.name

    def _format_type_name(self,
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
            raise ValueError(
                'Constructor and method parameters cannot both be True')

        formatted_type_name = ''
        name = type_name.name

        if include_namespace:
            for namespace in type_name.namespaces:
                if name not in self.ignore_namespace and namespace != '':
                    formatted_type_name += namespace + separator

        if constructor:
            formatted_type_name += self.data_type.get(name) or name
        elif method:
            formatted_type_name += self.data_type_param.get(name) or name
        else:
            formatted_type_name += name

        if separator == "::":  # C++
            templates = []
            for idx in range(len(type_name.instantiations)):
                template = '{}'.format(
                    self._format_type_name(type_name.instantiations[idx],
                                           include_namespace=include_namespace,
                                           constructor=constructor,
                                           method=method))
                templates.append(template)

            if len(templates) > 0:  # If there are no templates
                formatted_type_name += '<{}>'.format(','.join(templates))

        else:
            for idx in range(len(type_name.instantiations)):
                formatted_type_name += '{}'.format(
                    self._format_type_name(type_name.instantiations[idx],
                                           separator=separator,
                                           include_namespace=False,
                                           constructor=constructor,
                                           method=method))

        return formatted_type_name

    def _format_return_type(self,
                            return_type,
                            include_namespace=False,
                            separator="::"):
        """Format return_type.

        Args:
            return_type: an interface_parser.ReturnType to reformat
            include_namespace: whether to include namespaces when reformatting
        """
        return_wrap = ''

        if self._return_count(return_type) == 1:
            return_wrap = self._format_type_name(
                return_type.type1.typename,
                separator=separator,
                include_namespace=include_namespace)
        else:
            return_wrap = 'pair< {type1}, {type2} >'.format(
                type1=self._format_type_name(
                    return_type.type1.typename,
                    separator=separator,
                    include_namespace=include_namespace),
                type2=self._format_type_name(
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
