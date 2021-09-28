"""Code to help instantiate templated classes, methods and functions."""

# pylint: disable=too-many-arguments, too-many-instance-attributes, no-self-use, no-else-return, too-many-arguments, unused-format-string-argument, unused-variable

import itertools
from copy import deepcopy
from typing import Any, Iterable, List, Sequence

import gtwrap.interface_parser as parser


def is_scoped_template(template_typenames, str_arg_typename):
    """
    Check if the template given by `str_arg_typename` is a scoped template,
    and if so, return what template and index matches the scoped template correctly.
    """
    for idx, template in enumerate(template_typenames):
        if template in str_arg_typename.split("::"):
            return template, idx
    return False, -1


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
        new_type = instantiate_type(arg.ctype, template_typenames,
                                    instantiations, cpp_typename)
        instantiated_args.append(
            parser.Argument(name=arg.name, ctype=new_type,
                            default=arg.default))
    return instantiated_args


def instantiate_return_type(return_type,
                            template_typenames,
                            instantiations,
                            cpp_typename,
                            instantiated_class=None):
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
                original.args.list(),
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
            instantiated_names = [
                inst.instantiated_name() for inst in self.instantiations
            ]
            ret = "{}<{}>".format(self.original.name,
                                  ",".join(instantiated_names))
        else:
            ret = self.original.name
        return ret

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedGlobalFunction, self).__repr__())


class InstantiatedMethod(parser.Method):
    """
    Instantiate method with template parameters.

    E.g.
    class A {
        template<X, Y>
        void func(X x, Y y);
    }
    """
    def __init__(self,
                 original: parser.Method,
                 instantiations: Iterable[parser.Typename] = ()):
        self.original = original
        self.instantiations = instantiations
        self.template: Any = ''
        self.is_const = original.is_const
        self.parent = original.parent

        # Check for typenames if templated.
        # This way, we can gracefully handle both templated and non-templated methods.
        typenames: Sequence = self.original.template.typenames if self.original.template else []
        self.name = instantiate_name(original.name, self.instantiations)
        self.return_type = instantiate_return_type(
            original.return_type,
            typenames,
            self.instantiations,
            # Keyword type name `This` should already be replaced in the
            # previous class template instantiation round.
            cpp_typename='',
        )

        instantiated_args = instantiate_args_list(
            original.args.list(),
            typenames,
            self.instantiations,
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
            # to_cpp will handle all the namespacing and templating
            instantiation_list = [x.to_cpp() for x in self.instantiations]
            # now can simply combine the instantiations, separated by commas
            ret = "{}<{}>".format(self.original.name,
                                  ",".join(instantiation_list))
        else:
            ret = self.original.name
        return ret

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedMethod, self).__repr__())


class InstantiatedClass(parser.Class):
    """
    Instantiate the class defined in the interface file.
    """
    def __init__(self, original: parser.Class, instantiations=(), new_name=''):
        """
        Template <T, U>
        Instantiations: [T1, U1]
        """
        self.original = original
        self.instantiations = instantiations

        self.template = None
        self.is_virtual = original.is_virtual
        self.parent_class = original.parent_class
        self.parent = original.parent

        # If the class is templated, check if the number of provided instantiations
        # match the number of templates, else it's only a partial instantiation which is bad.
        if original.template:
            assert len(original.template.typenames) == len(
                instantiations), "Typenames and instantiations mismatch!"

        # Get the instantiated name of the class. E.g. FuncDouble
        self.name = instantiate_name(
            original.name, instantiations) if not new_name else new_name

        # Check for typenames if templated.
        # By passing in typenames, we can gracefully handle both templated and non-templated classes
        # This will allow the `This` keyword to be used in both templated and non-templated classes.
        typenames = self.original.template.typenames if self.original.template else []

        # Instantiate the constructors, static methods, properties, respectively.
        self.ctors = self.instantiate_ctors(typenames)
        self.static_methods = self.instantiate_static_methods(typenames)
        self.properties = self.instantiate_properties(typenames)

        # Instantiate all operator overloads
        self.operators = self.instantiate_operators(typenames)

        # Set enums
        self.enums = original.enums

        # Instantiate all instance methods
        instantiated_methods = \
            self.instantiate_class_templates_in_methods(typenames)

        # Second instantiation round to instantiate templated methods.
        # This is done in case both the class and the method are templated.
        self.methods = []
        for method in instantiated_methods:
            if not method.template:
                self.methods.append(InstantiatedMethod(method, ()))
            else:
                instantiations = []
                # Get all combinations of template parameters
                for instantiations in itertools.product(
                        *method.template.instantiations):
                    self.methods.append(
                        InstantiatedMethod(method, instantiations))

        super().__init__(
            self.template,
            self.is_virtual,
            self.name,
            [self.parent_class],
            self.ctors,
            self.methods,
            self.static_methods,
            self.properties,
            self.operators,
            self.enums,
            parent=self.parent,
        )

    def __repr__(self):
        return "{virtual}Class {cpp_class} : {parent_class}\n"\
            "{ctors}\n{static_methods}\n{methods}\n{operators}".format(
               virtual="virtual " if self.is_virtual else '',
               cpp_class=self.to_cpp(),
               parent_class=self.parent,
               ctors="\n".join([repr(ctor) for ctor in self.ctors]),
               static_methods="\n".join([repr(m)
                                         for m in self.static_methods]),
                methods="\n".join([repr(m) for m in self.methods]),
               operators="\n".join([repr(op) for op in self.operators])
            )

    def instantiate_ctors(self, typenames):
        """
        Instantiate the class constructors.

        Args:
            typenames: List of template types to instantiate.

        Return: List of constructors instantiated with provided template args.
        """
        instantiated_ctors = []

        def instantiate(instantiated_ctors, ctor, typenames, instantiations):
            instantiated_args = instantiate_args_list(
                ctor.args.list(),
                typenames,
                instantiations,
                self.cpp_typename(),
            )
            instantiated_ctors.append(
                parser.Constructor(
                    name=self.name,
                    args=parser.ArgumentList(instantiated_args),
                    template=self.original.template,
                    parent=self,
                ))
            return instantiated_ctors

        for ctor in self.original.ctors:
            # Add constructor templates to the typenames and instantiations
            if isinstance(ctor.template, parser.template.Template):
                typenames.extend(ctor.template.typenames)

                # Get all combinations of template args
                for instantiations in itertools.product(
                        *ctor.template.instantiations):
                    instantiations = self.instantiations + list(instantiations)

                    instantiated_ctors = instantiate(
                        instantiated_ctors,
                        ctor,
                        typenames=typenames,
                        instantiations=instantiations)

            else:
                # If no constructor level templates, just use the class templates
                instantiated_ctors = instantiate(
                    instantiated_ctors,
                    ctor,
                    typenames=typenames,
                    instantiations=self.instantiations)

        return instantiated_ctors

    def instantiate_static_methods(self, typenames):
        """
        Instantiate static methods in the class.

        Args:
            typenames: List of template types to instantiate.

        Return: List of static methods instantiated with provided template args.
        """
        instantiated_static_methods = []
        for static_method in self.original.static_methods:
            instantiated_args = instantiate_args_list(
                static_method.args.list(), typenames, self.instantiations,
                self.cpp_typename())
            instantiated_static_methods.append(
                parser.StaticMethod(
                    name=static_method.name,
                    return_type=instantiate_return_type(
                        static_method.return_type,
                        typenames,
                        self.instantiations,
                        self.cpp_typename(),
                        instantiated_class=self),
                    args=parser.ArgumentList(instantiated_args),
                    parent=self,
                ))
        return instantiated_static_methods

    def instantiate_class_templates_in_methods(self, typenames):
        """
        This function only instantiates the class-level templates in the methods.
        Template methods are instantiated in InstantiatedMethod in the second
        round.

        E.g.
        ```
        template<T={string}>
        class Greeter{
            void sayHello(T& name);
        };

        Args:
            typenames: List of template types to instantiate.

        Return: List of methods instantiated with provided template args on the class.
        """
        class_instantiated_methods = []
        for method in self.original.methods:
            instantiated_args = instantiate_args_list(
                method.args.list(),
                typenames,
                self.instantiations,
                self.cpp_typename(),
            )
            class_instantiated_methods.append(
                parser.Method(
                    template=method.template,
                    name=method.name,
                    return_type=instantiate_return_type(
                        method.return_type,
                        typenames,
                        self.instantiations,
                        self.cpp_typename(),
                    ),
                    args=parser.ArgumentList(instantiated_args),
                    is_const=method.is_const,
                    parent=self,
                ))
        return class_instantiated_methods

    def instantiate_operators(self, typenames):
        """
        Instantiate the class-level template in the operator overload.

        Args:
            typenames: List of template types to instantiate.

        Return: List of methods instantiated with provided template args on the class.
        """
        instantiated_operators = []
        for operator in self.original.operators:
            instantiated_args = instantiate_args_list(
                operator.args.list(),
                typenames,
                self.instantiations,
                self.cpp_typename(),
            )
            instantiated_operators.append(
                parser.Operator(
                    name=operator.name,
                    operator=operator.operator,
                    return_type=instantiate_return_type(
                        operator.return_type,
                        typenames,
                        self.instantiations,
                        self.cpp_typename(),
                    ),
                    args=parser.ArgumentList(instantiated_args),
                    is_const=operator.is_const,
                    parent=self,
                ))
        return instantiated_operators

    def instantiate_properties(self, typenames):
        """
        Instantiate the class properties.

        Args:
            typenames: List of template types to instantiate.

        Return: List of properties instantiated with provided template args.
        """
        instantiated_properties = instantiate_args_list(
            self.original.properties,
            typenames,
            self.instantiations,
            self.cpp_typename(),
        )
        return instantiated_properties

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

    def to_cpp(self):
        """Generate the C++ code for wrapping."""
        return self.cpp_typename().to_cpp()


class InstantiatedDeclaration(parser.ForwardDeclaration):
    """
    Instantiate typedefs of forward declarations.
    This is useful when we wish to typedef a templated class
    which is not defined in the current project.

    E.g.
        class FactorFromAnotherMother;

        typedef FactorFromAnotherMother<gtsam::Pose3> FactorWeCanUse;
    """
    def __init__(self, original, instantiations=(), new_name=''):
        super().__init__(original.typename,
                         original.parent_type,
                         original.is_virtual,
                         parent=original.parent)

        self.original = original
        self.instantiations = instantiations
        self.parent = original.parent

        self.name = instantiate_name(
            original.name, instantiations) if not new_name else new_name

    def to_cpp(self):
        """Generate the C++ code for wrapping."""
        instantiated_names = [
            inst.qualified_name() for inst in self.instantiations
        ]
        name = "{}<{}>".format(self.original.name,
                               ",".join(instantiated_names))
        namespaces_name = self.namespaces()
        namespaces_name.append(name)
        # Leverage Typename to generate the fully qualified C++ name
        return parser.Typename(namespaces_name).to_cpp()

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedDeclaration, self).__repr__())


def instantiate_namespace(namespace):
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
                # This case is for when the templates have enumerated instantiations.

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
            # This is for the case where `typedef` statements are used
            # to specify the template parameters.
            typedef_inst = element
            top_level = namespace.top_level()
            original_element = top_level.find_class_or_function(
                typedef_inst.typename)

            # Check if element is a typedef'd class, function or
            # forward declaration from another project.
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
            elif isinstance(original_element, parser.ForwardDeclaration):
                typedef_content.append(
                    InstantiatedDeclaration(
                        original_element, typedef_inst.typename.instantiations,
                        typedef_inst.new_name))

        elif isinstance(element, parser.Namespace):
            element = instantiate_namespace(element)
            instantiated_content.append(element)
        else:
            instantiated_content.append(element)

    instantiated_content.extend(typedef_content)
    namespace.content = instantiated_content

    return namespace
