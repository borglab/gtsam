"""Mixins for reducing the amount of boilerplate in the main wrapper class."""

from typing import Any, Tuple, Union

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator


class CheckMixin:
    """Mixin to provide various checks."""
    # Data types that are primitive types
    not_ptr_type: Tuple = ('int', 'double', 'bool', 'char', 'unsigned char',
                           'size_t')
    # Ignore the namespace for these datatypes
    ignore_namespace: Tuple = ('Matrix', 'Vector', 'Point2', 'Point3')
    # Methods that should be ignored
    ignore_methods: Tuple = ('pickle', )
    # Methods that should not be wrapped directly
    whitelist: Tuple = ('serializable', 'serialize')
    # Datatypes that do not need to be checked in methods
    not_check_type: list = []

    def _has_serialization(self, cls):
        for m in cls.methods:
            if m.name in self.whitelist:
                return True
        return False

    def can_be_pointer(self, arg_type: parser.Type):
        """
        Determine if the `arg_type` can have a pointer to it.

        E.g. `Pose3` can have `Pose3*` but 
        `Matrix` should not have `Matrix*`.
        """
        return (arg_type.typename.name not in self.not_ptr_type
                and arg_type.typename.name not in self.ignore_namespace
                and arg_type.typename.name != 'string')

    def is_shared_ptr(self, arg_type: parser.Type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        shared pointer in the wrapper.
        """
        return arg_type.is_shared_ptr

    def is_ptr(self, arg_type: parser.Type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        raw pointer in the wrapper.
        """
        return arg_type.is_ptr

    def is_ref(self, arg_type: parser.Type):
        """
        Determine if the `interface_parser.Type` should be treated as a
        reference in the wrapper.
        """
        return arg_type.typename.name not in self.ignore_namespace and \
               arg_type.typename.name not in self.not_ptr_type and \
               arg_type.is_ref

    def is_class_enum(self, arg_type: parser.Type, class_: parser.Class):
        """Check if arg_type is an enum in the class `class_`."""
        if class_:
            class_enums = [enum.name for enum in class_.enums]
            return arg_type.typename.name in class_enums
        else:
            return False

    def is_global_enum(self, arg_type: parser.Type, class_: parser.Class):
        """Check if arg_type is a global enum."""
        if class_:
            # Get the enums in the class' namespace
            global_enums = [
                member.name for member in class_.parent.content
                if isinstance(member, parser.Enum)
            ]
            return arg_type.typename.name in global_enums
        else:
            return False

    def is_enum(self, arg_type: parser.Type, class_: parser.Class):
        """Check if `arg_type` is an enum."""
        return self.is_class_enum(arg_type, class_) or self.is_global_enum(
            arg_type, class_)


class FormatMixin:
    """Mixin to provide formatting utilities."""

    ignore_namespace: tuple
    data_type: Any
    data_type_param: Any
    _return_count: Any

    def _clean_class_name(self,
                          instantiated_class: instantiator.InstantiatedClass):
        """Reformatted the C++ class name to fit Matlab defined naming
        standards
        """
        if len(instantiated_class.ctors) != 0:
            return instantiated_class.ctors[0].name

        return instantiated_class.name

    def _format_type_name(self,
                          type_name: parser.Typename,
                          separator: str = '::',
                          include_namespace: bool = True,
                          is_constructor: bool = False,
                          is_method: bool = False):
        """
        Args:
            type_name: an interface_parser.Typename to reformat
            separator: the statement to add between namespaces and typename
            include_namespace: whether to include namespaces when reformatting
            is_constructor: if the typename will be in a constructor
            is_method: if the typename will be in a method

        Raises:
            constructor and method cannot both be true
        """
        if is_constructor and is_method:
            raise ValueError(
                'Constructor and method parameters cannot both be True')

        formatted_type_name = ''
        name = type_name.name

        if include_namespace:
            for namespace in type_name.namespaces:
                if name not in self.ignore_namespace and namespace != '':
                    formatted_type_name += namespace + separator

        if is_constructor:
            formatted_type_name += self.data_type.get(name) or name
        elif is_method:
            formatted_type_name += self.data_type_param.get(name) or name
        else:
            formatted_type_name += str(name)

        if separator == "::":  # C++
            templates = []
            for idx, _ in enumerate(type_name.instantiations):
                template = '{}'.format(
                    self._format_type_name(type_name.instantiations[idx],
                                           include_namespace=include_namespace,
                                           is_constructor=is_constructor,
                                           is_method=is_method))
                templates.append(template)

            if len(templates) > 0:  # If there are no templates
                formatted_type_name += '<{}>'.format(','.join(templates))

        else:
            for idx, _ in enumerate(type_name.instantiations):
                formatted_type_name += '{}'.format(
                    self._format_type_name(type_name.instantiations[idx],
                                           separator=separator,
                                           include_namespace=False,
                                           is_constructor=is_constructor,
                                           is_method=is_method))

        return formatted_type_name

    def _format_return_type(self,
                            return_type: parser.function.ReturnType,
                            include_namespace: bool = False,
                            separator: str = "::"):
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

    def _format_class_name(self,
                           instantiated_class: instantiator.InstantiatedClass,
                           separator: str = ''):
        """Format a template_instantiator.InstantiatedClass name."""
        if instantiated_class.parent == '':
            parent_full_ns = ['']
        else:
            parent_full_ns = instantiated_class.parent.full_namespaces()

        parentname = "".join([separator + x
                              for x in parent_full_ns]) + separator

        class_name = parentname[2 * len(separator):]

        class_name += instantiated_class.name

        return class_name

    def _format_static_method(self,
                              static_method: parser.StaticMethod,
                              separator: str = ''):
        """
        Example:
                gtsam.Point3.staticFunction()
        """
        method = ''

        if isinstance(static_method, parser.StaticMethod):
            method += static_method.parent.to_cpp() + separator

        return method

    def _format_global_function(self,
                                function: Union[parser.GlobalFunction, Any],
                                separator: str = ''):
        """Example:

                gtsamPoint3.staticFunction
        """
        method = ''

        if isinstance(function, parser.GlobalFunction):
            method += "".join([separator + x for x in function.parent.full_namespaces()]) + \
                      separator

        return method[2 * len(separator):]
