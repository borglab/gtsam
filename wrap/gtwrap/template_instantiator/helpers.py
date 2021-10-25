"""Various helpers for instantiation."""

import itertools
from copy import deepcopy
from typing import List, Sequence, Union

import gtwrap.interface_parser as parser

ClassMembers = Union[parser.Constructor, parser.Method, parser.StaticMethod,
                     parser.GlobalFunction, parser.Operator, parser.Variable,
                     parser.Enum]
InstantiatedMembers = Union['InstantiatedConstructor', 'InstantiatedMethod',
                            'InstantiatedStaticMethod',
                            'InstantiatedGlobalFunction']


def is_scoped_template(template_typenames: Sequence[str],
                       str_arg_typename: str):
    """
    Check if the template given by `str_arg_typename` is a scoped template e.g. T::Value,
    and if so, return what template from `template_typenames` and
    the corresponding index matches the scoped template correctly.
    """
    for idx, template in enumerate(template_typenames):
        if "::" in str_arg_typename and \
            template in str_arg_typename.split("::"):
            return template, idx
    return False, -1


def instantiate_type(
        ctype: parser.Type,
        template_typenames: Sequence[str],
        instantiations: Sequence[parser.Typename],
        cpp_typename: parser.Typename,
        instantiated_class: 'InstantiatedClass' = None) -> parser.Type:
    """
    Instantiate template typename for `ctype`.

    Args:
        ctype: The original argument type.
        template_typenames: List of strings representing the templates.
        instantiations: List of the instantiations of the templates in `template_typenames`.
        cpp_typename: Full-namespace cpp class name of this instantiation
            to replace for arguments of type named `This`.
        instiated_class: The instantiated class which, if provided,
            will be used for instantiating `This`.

    Returns:
        If `ctype`'s name is in the `template_typenames`, return the
        corresponding type to replace in `instantiations`.
        If ctype name is `This`, return the new typename `cpp_typename`.
        Otherwise, return the original ctype.
    """
    # make a deep copy so that there is no overwriting of original template params
    ctype = deepcopy(ctype)

    # Check if the return type has template parameters
    if ctype.typename.instantiations:
        for idx, instantiation in enumerate(ctype.typename.instantiations):
            if instantiation.name in template_typenames:
                template_idx = template_typenames.index(instantiation.name)
                ctype.typename.instantiations[
                    idx] = instantiations[  # type: ignore
                        template_idx]

        return ctype

    str_arg_typename = str(ctype.typename)

    # Check if template is a scoped template e.g. T::Value where T is the template
    scoped_template, scoped_idx = is_scoped_template(template_typenames,
                                                     str_arg_typename)

    # Instantiate templates which have enumerated instantiations in the template.
    # E.g. `template<T={double}>`.

    # Instantiate scoped templates, e.g. T::Value.
    if scoped_template:
        # Create a copy of the instantiation so we can modify it.
        instantiation = deepcopy(instantiations[scoped_idx])
        # Replace the part of the template with the instantiation
        instantiation.name = str_arg_typename.replace(scoped_template,
                                                      instantiation.name)
        return parser.Type(
            typename=instantiation,
            is_const=ctype.is_const,
            is_shared_ptr=ctype.is_shared_ptr,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basic=ctype.is_basic,
        )
    # Check for exact template match.
    elif str_arg_typename in template_typenames:
        idx = template_typenames.index(str_arg_typename)
        return parser.Type(
            typename=instantiations[idx],
            is_const=ctype.is_const,
            is_shared_ptr=ctype.is_shared_ptr,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basic=ctype.is_basic,
        )

    # If a method has the keyword `This`, we replace it with the (instantiated) class.
    elif str_arg_typename == 'This':
        # Check if the class is template instantiated
        # so we can replace it with the instantiated version.
        if instantiated_class:
            name = instantiated_class.original.name
            namespaces_name = instantiated_class.namespaces()
            namespaces_name.append(name)
            cpp_typename = parser.Typename(
                namespaces_name,
                instantiations=instantiated_class.instantiations)

        return parser.Type(
            typename=cpp_typename,
            is_const=ctype.is_const,
            is_shared_ptr=ctype.is_shared_ptr,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basic=ctype.is_basic,
        )

    # Case when 'This' is present in the type namespace, e.g `This::Subclass`.
    elif 'This' in str_arg_typename:
        # Simply get the index of `This` in the namespace and replace it with the instantiated name.
        namespace_idx = ctype.typename.namespaces.index('This')
        ctype.typename.namespaces[namespace_idx] = cpp_typename.name
        return ctype

    else:
        return ctype


def instantiate_args_list(
        args_list: Sequence[parser.Argument],
        template_typenames: Sequence[parser.template.Typename],
        instantiations: Sequence, cpp_typename: parser.Typename):
    """
    Instantiate template typenames in an argument list.
    Type with name `This` will be replaced by @p `cpp_typename`.

    @param[in] args_list A list of `parser.Argument` to instantiate.
    @param[in] template_typenames List of template typenames to instantiate,
        e.g. ['T', 'U', 'V'].
    @param[in] instantiations List of specific types to instantiate, each
        associated with each template typename. Each type is a parser.Typename,
        including its name and full namespaces.
    @param[in] cpp_typename Full-namespace cpp class name of this instantiation
        to replace for arguments of type named `This`.
    @return A new list of parser.Argument which types are replaced with their
        instantiations.
    """
    instantiated_args = []
    for arg in args_list:
        new_type = instantiate_type(arg.ctype, template_typenames,
                                    instantiations, cpp_typename)
        instantiated_args.append(
            parser.Argument(name=arg.name, ctype=new_type,
                            default=arg.default))
    return instantiated_args


def instantiate_return_type(
        return_type: parser.ReturnType,
        template_typenames: Sequence[parser.template.Typename],
        instantiations: Sequence[parser.Typename],
        cpp_typename: parser.Typename,
        instantiated_class: 'InstantiatedClass' = None):
    """Instantiate the return type."""
    new_type1 = instantiate_type(return_type.type1,
                                 template_typenames,
                                 instantiations,
                                 cpp_typename,
                                 instantiated_class=instantiated_class)
    if return_type.type2:
        new_type2 = instantiate_type(return_type.type2,
                                     template_typenames,
                                     instantiations,
                                     cpp_typename,
                                     instantiated_class=instantiated_class)
    else:
        new_type2 = ''
    return parser.ReturnType(new_type1, new_type2)


def instantiate_name(original_name: str,
                     instantiations: Sequence[parser.Typename]):
    """
    Concatenate instantiated types with `original_name` to form a new
    instantiated name.

    NOTE: To avoid conflicts, we should include the instantiation's
    namespaces, but that is too verbose.
    """
    instantiated_names = []
    for inst in instantiations:
        # Ensure the first character of the type is capitalized
        name = inst.instantiated_name()
        # Using `capitalize` on the complete name causes other caps to be lower case
        instantiated_names.append(name.replace(name[0], name[0].capitalize()))

    return "{}{}".format(original_name, "".join(instantiated_names))


class InstantiationHelper:
    """
    Helper class for instantiation templates.
    Requires that `instantiation_type` defines a class method called
    `construct` to generate the appropriate object type.

    Signature for `construct` should be
    ```
        construct(method,
                  typenames,
                  class_instantiations,
                  method_instantiations,
                  instantiated_args,
                  parent=parent)
    ```
    """
    def __init__(self, instantiation_type: InstantiatedMembers):
        self.instantiation_type = instantiation_type

    def instantiate(self, instantiated_methods: List[InstantiatedMembers],
                    method: ClassMembers, typenames: Sequence[str],
                    class_instantiations: Sequence[parser.Typename],
                    method_instantiations: Sequence[parser.Typename],
                    parent: 'InstantiatedClass'):
        """
        Instantiate both the class and method level templates.
        """
        instantiations = class_instantiations + method_instantiations

        instantiated_args = instantiate_args_list(method.args.list(),
                                                  typenames, instantiations,
                                                  parent.cpp_typename())

        instantiated_methods.append(
            self.instantiation_type.construct(method,
                                              typenames,
                                              class_instantiations,
                                              method_instantiations,
                                              instantiated_args,
                                              parent=parent))

        return instantiated_methods

    def multilevel_instantiation(self, methods_list: Sequence[ClassMembers],
                                 typenames: Sequence[str],
                                 parent: 'InstantiatedClass'):
        """
        Helper to instantiate methods at both the class and method level.

        Args:
            methods_list: The list of methods in the class to instantiated.
            typenames: List of class level template parameters, e.g. ['T'].
            parent: The instantiated class to which `methods_list` belongs.
        """
        instantiated_methods = []

        for method in methods_list:
            # We creare a copy since we will modify the typenames list.
            method_typenames = deepcopy(typenames)

            if isinstance(method.template, parser.template.Template):
                method_typenames.extend(method.template.typenames)

                # Get all combinations of template args
                for instantiations in itertools.product(
                        *method.template.instantiations):

                    instantiated_methods = self.instantiate(
                        instantiated_methods,
                        method,
                        typenames=method_typenames,
                        class_instantiations=parent.instantiations,
                        method_instantiations=list(instantiations),
                        parent=parent)

            else:
                # If no constructor level templates, just use the class templates
                instantiated_methods = self.instantiate(
                    instantiated_methods,
                    method,
                    typenames=method_typenames,
                    class_instantiations=parent.instantiations,
                    method_instantiations=[],
                    parent=parent)

        return instantiated_methods
