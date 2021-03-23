"""Code to help instantiate templated classes, methods and functions."""

# pylint: disable=too-many-arguments, too-many-instance-attributes, no-self-use, no-else-return, too-many-arguments, unused-format-string-argument, unused-variable

import itertools
from copy import deepcopy
from typing import List

import gtwrap.interface_parser as parser


def instantiate_type(ctype: parser.Type,
                     template_typenames: List[str],
                     instantiations: List[parser.Typename],
                     cpp_typename: parser.Typename,
                     instantiated_class=None):
    """
    Instantiate template typename for @p ctype.

    Args:
        instiated_class (InstantiatedClass):

    @return If ctype's name is in the @p template_typenames, return the
        corresponding type to replace in @p instantiations.
        If ctype name is `This`, return the new typename @p `cpp_typename`.
        Otherwise, return the original ctype.
    """
    # make a deep copy so that there is no overwriting of original template params
    ctype = deepcopy(ctype)

    # Check if the return type has template parameters
    if len(ctype.typename.instantiations) > 0:
        for idx, instantiation in enumerate(ctype.typename.instantiations):
            if instantiation.name in template_typenames:
                template_idx = template_typenames.index(instantiation.name)
                ctype.typename.instantiations[idx] = instantiations[
                    template_idx]

        return ctype

    str_arg_typename = str(ctype.typename)

    if str_arg_typename in template_typenames:
        idx = template_typenames.index(str_arg_typename)
        return parser.Type(
            typename=instantiations[idx],
            is_const=ctype.is_const,
            is_shared_ptr=ctype.is_shared_ptr,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basis=ctype.is_basis,
        )
    elif str_arg_typename == 'This':
        if instantiated_class:
            name = instantiated_class.original.name
            namespaces_name = instantiated_class.namespaces()
            namespaces_name.append(name)
            # print("INST: {}, {}, CPP: {}, CLS: {}".format(
            #     ctype, instantiations, cpp_typename, instantiated_class.instantiations
            # ), file=sys.stderr)
            cpp_typename = parser.Typename(
                namespaces_name,
                instantiations=instantiated_class.instantiations)
        return parser.Type(
            typename=cpp_typename,
            is_const=ctype.is_const,
            is_shared_ptr=ctype.is_shared_ptr,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basis=ctype.is_basis,
        )
    else:
        return ctype


def instantiate_args_list(args_list, template_typenames, instantiations,
                          cpp_typename):
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
        new_type = instantiate_type(
            arg.ctype, template_typenames, instantiations, cpp_typename)
        instantiated_args.append(
            parser.Argument(name=arg.name, ctype=new_type))
    return instantiated_args


def instantiate_return_type(return_type, template_typenames, instantiations,
                            cpp_typename, instantiated_class=None):
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


def instantiate_name(original_name, instantiations):
    """
    Concatenate instantiated types with an @p original name to form a new
    instantiated name.
    TODO(duy): To avoid conflicts, we should include the instantiation's
    namespaces, but I find that too verbose.
    """
    inst_name = ''
    instantiated_names = []
    for inst in instantiations:
        # Ensure the first character of the type is capitalized
        name = inst.instantiated_name()
        # Using `capitalize` on the complete name causes other caps to be lower case
        instantiated_names.append(name.replace(name[0], name[0].capitalize()))

    return "{}{}".format(original_name, "".join(instantiated_names))

class InstantiatedGlobalFunction(parser.GlobalFunction):
    """
    Instantiate global functions.

    E.g.
        template<T = {double}>
        T add(const T& x, const T& y);
    """
    def __init__(self, original, instantiations=(), new_name=''):
        self.original = original
        self.instantiations = instantiations
        self.template = ''
        self.parent = original.parent

        if not original.template:
            self.name = original.name
            self.return_type = original.return_type
            self.args = original.args
        else:
            self.name = instantiate_name(
                original.name, instantiations) if not new_name else new_name
            self.return_type = instantiate_return_type(
                original.return_type,
                self.original.template.typenames,
                self.instantiations,
                # Keyword type name `This` should already be replaced in the
                # previous class template instantiation round.
                cpp_typename='',
            )
            instantiated_args = instantiate_args_list(
                original.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                # Keyword type name `This` should already be replaced in the
                # previous class template instantiation round.
                cpp_typename='',
            )
            self.args = parser.ArgumentList(instantiated_args)

        super().__init__(self.name,
                         self.return_type,
                         self.args,
                         self.template,
                         parent=self.parent)

    def to_cpp(self):
        """Generate the C++ code for wrapping."""
        if self.original.template:
            instantiated_names = [inst.instantiated_name() for inst in self.instantiations]
            ret = "{}<{}>".format(self.original.name, ",".join(instantiated_names))
        else:
            ret = self.original.name
        return ret

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedGlobalFunction, self).__repr__()
        )

class InstantiatedMethod(parser.Method):
    """
    We can only instantiate template methods with a single template parameter.
    """

    def __init__(self, original, instantiation=''):
        self.original = original
        self.instantiation = instantiation
        self.template = ''
        self.is_const = original.is_const
        self.parent = original.parent

        if not original.template:
            self.name = original.name
            self.return_type = original.return_type
            self.args = original.args
        else:
            #TODO(Varun) enable multiple templates for methods
            if len(self.original.template.typenames) > 1:
                raise ValueError("Can only instantiate template method with "
                                 "single template parameter.")
            self.name = instantiate_name(original.name, [self.instantiation])
            self.return_type = instantiate_return_type(
                original.return_type,
                [self.original.template.typenames[0]],
                [self.instantiation],
                # Keyword type name `This` should already be replaced in the
                # previous class template instantiation round.
                cpp_typename='',
            )
            instantiated_args = instantiate_args_list(
                original.args.args_list,
                [self.original.template.typenames[0]],
                [self.instantiation],
                # Keyword type name `This` should already be replaced in the
                # previous class template instantiation round.
                cpp_typename='',
            )
            self.args = parser.ArgumentList(instantiated_args)

        super().__init__(self.template,
                         self.name,
                         self.return_type,
                         self.args,
                         self.is_const,
                         parent=self.parent)

    def to_cpp(self):
        """Generate the C++ code for wrapping."""
        if self.original.template:
            ret = "{}<{}>".format(self.original.name, self.instantiation)
        else:
            ret = self.original.name
        return ret

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedMethod, self).__repr__()
        )


class InstantiatedClass(parser.Class):
    """
    Instantiate the class defined in the interface file.
    """
    def __init__(self, original, instantiations=(), new_name=''):
        """
        Template <T, U>
        Instantiations: [T1, U1]
        """
        self.original = original
        self.instantiations = instantiations

        self.template = ''
        self.is_virtual = original.is_virtual
        self.parent_class = original.parent_class
        self.parent = original.parent

        if not original.template:
            self.name = original.name
            self.ctors = list(original.ctors)
            self.static_methods = list(original.static_methods)
            class_instantiated_methods = list(original.methods)
            self.properties = list(original.properties)
        else:
            # Check conditions.
            assert len(original.template.typenames) == len(
                instantiations), "Typenames and instantiations mismatch!"

            self.name = instantiate_name(
                original.name, instantiations) if not new_name else new_name
            self.ctors = self.instantiate_ctors()
            self.static_methods = self.instantiate_static_methods()
            class_instantiated_methods = \
                self.instantiate_class_templates_in_methods()
            self.properties = self.instantiate_properties()

        # Second instantiation round to instantiate template methods.
        self.methods = []
        for method in class_instantiated_methods:
            if not method.template:
                self.methods.append(InstantiatedMethod(method, ''))
            else:
                assert len(
                    method.template.typenames) == 1, ""\
                    "Can only instantiate single template methods"
                for inst in method.template.instantiations[0]:
                    self.methods.append(InstantiatedMethod(method, inst))

        super().__init__(
            self.template,
            self.is_virtual,
            self.name,
            [self.parent_class],
            self.ctors,
            self.methods,
            self.static_methods,
            self.properties,
            parent=self.parent,
        )

    def __repr__(self):
        return "{virtual} class {name} [{cpp_class}]: {parent_class}\n"\
            "{ctors}\n{static_methods}\n{methods}".format(
               virtual="virtual" if self.is_virtual else '',
               name=self.name,
               cpp_class=self.cpp_class(),
               parent_class=self.parent,
               ctors="\n".join([ctor.__repr__() for ctor in self.ctors]),
               methods="\n".join([m.__repr__() for m in self.methods]),
               static_methods="\n".join([m.__repr__()
                                         for m in self.static_methods]),
            )

    def instantiate_ctors(self):
        """Instantiate the class constructors."""
        instantiated_ctors = []
        for ctor in self.original.ctors:
            instantiated_args = instantiate_args_list(
                ctor.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                self.cpp_typename(),
            )
            instantiated_ctors.append(parser.Constructor(
                name=self.name,
                args=parser.ArgumentList(instantiated_args),
                parent=self,
            ))
        return instantiated_ctors

    def instantiate_static_methods(self):
        """Instantiate static methods in the class."""
        instantiated_static_methods = []
        for static_method in self.original.static_methods:
            instantiated_args = instantiate_args_list(
                static_method.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                self.cpp_typename()
            )
            instantiated_static_methods.append(
                parser.StaticMethod(
                    name=static_method.name,
                    return_type=instantiate_return_type(
                        static_method.return_type,
                        self.original.template.typenames,
                        self.instantiations,
                        self.cpp_typename(),
                        instantiated_class=self
                    ),
                    args=parser.ArgumentList(instantiated_args),
                    parent=self,
                )
            )
        return instantiated_static_methods

    def instantiate_class_templates_in_methods(self):
        """
        This function only instantiates class templates in the methods.
        Template methods are instantiated in InstantiatedMethod in the second
        round.
        """
        class_instantiated_methods = []
        for method in self.original.methods:
            instantiated_args = instantiate_args_list(
                method.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                self.cpp_typename(),
            )
            class_instantiated_methods.append(parser.Method(
                template=method.template,
                name=method.name,
                return_type=instantiate_return_type(
                    method.return_type,
                    self.original.template.typenames,
                    self.instantiations,
                    self.cpp_typename(),
                ),
                args=parser.ArgumentList(instantiated_args),
                is_const=method.is_const,
                parent=self,
            ))
        return class_instantiated_methods

    def instantiate_properties(self):
        """Instantiate the class properties."""
        instantiated_properties = instantiate_args_list(
            self.original.properties,
            self.original.template.typenames,
            self.instantiations,
            self.cpp_typename(),
        )
        return instantiated_properties

    def cpp_class(self):
        """Generate the C++ code for wrapping."""
        return self.cpp_typename().to_cpp()

    def cpp_typename(self):
        """
        Return a parser.Typename including namespaces and cpp name of this
        class.
        """
        if self.original.template:
            name = "{}<{}>".format(
                self.original.name,
                ", ".join([inst.to_cpp() for inst in self.instantiations]))
        else:
            name = self.original.name
        namespaces_name = self.namespaces()
        namespaces_name.append(name)
        return parser.Typename(namespaces_name)


def instantiate_namespace_inplace(namespace):
    """
    Instantiate the classes and other elements in the `namespace` content and
    assign it back to the namespace content attribute.

    @param[in/out] namespace The namespace whose content will be replaced with
        the instantiated content.
    """
    instantiated_content = []
    typedef_content = []

    for element in namespace.content:
        if isinstance(element, parser.Class):
            original_class = element
            if not original_class.template:
                instantiated_content.append(
                    InstantiatedClass(original_class, []))
            else:
                # Use itertools to get all possible combinations of instantiations
                # Works even if one template does not have an instantiation list
                for instantiations in itertools.product(
                        *original_class.template.instantiations):
                    instantiated_content.append(
                        InstantiatedClass(original_class,
                                          list(instantiations)))

        elif isinstance(element, parser.GlobalFunction):
            original_func = element
            if not original_func.template:
                instantiated_content.append(
                    InstantiatedGlobalFunction(original_func, []))
            else:
                # Use itertools to get all possible combinations of instantiations
                # Works even if one template does not have an instantiation list
                for instantiations in itertools.product(
                        *original_func.template.instantiations):
                    instantiated_content.append(
                        InstantiatedGlobalFunction(original_func,
                                                   list(instantiations)))

        elif isinstance(element, parser.TypedefTemplateInstantiation):
            typedef_inst = element
            top_level = namespace.top_level()
            original_element = top_level.find_class_or_function(
                typedef_inst.typename)

            # Check if element is a typedef'd class or function.
            if isinstance(original_element, parser.Class):
                typedef_content.append(
                    InstantiatedClass(original_element,
                                      typedef_inst.typename.instantiations,
                                      typedef_inst.new_name))
            elif isinstance(original_element, parser.GlobalFunction):
                typedef_content.append(
                    InstantiatedGlobalFunction(
                        original_element, typedef_inst.typename.instantiations,
                        typedef_inst.new_name))

        elif isinstance(element, parser.Namespace):
            instantiate_namespace_inplace(element)
            instantiated_content.append(element)
        else:
            instantiated_content.append(element)

    instantiated_content.extend(typedef_content)
    namespace.content = instantiated_content
