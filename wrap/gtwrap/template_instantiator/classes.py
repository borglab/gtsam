"""Instantiate a class and its members."""

import gtwrap.interface_parser as parser
from gtwrap.template_instantiator.constructor import InstantiatedConstructor
from gtwrap.template_instantiator.helpers import (InstantiationHelper,
                                                  instantiate_args_list,
                                                  instantiate_name,
                                                  instantiate_return_type,
                                                  instantiate_type)
from gtwrap.template_instantiator.method import (InstantiatedMethod,
                                                 InstantiatedStaticMethod)


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

        # Instantiate the parent class, constructors, static methods, properties, respectively.
        self.parent_class = self.instantiate_parent_class(typenames)
        self.ctors = self.instantiate_ctors(typenames)
        self.static_methods = self.instantiate_static_methods(typenames)
        self.properties = self.instantiate_properties(typenames)

        # Instantiate all operator overloads
        self.operators = self.instantiate_operators(typenames)

        # Set enums
        self.enums = original.enums

        # Instantiate all instance methods
        self.methods = self.instantiate_methods(typenames)
        
        self.dunder_methods = original.dunder_methods

        super().__init__(
            self.template,
            self.is_virtual,
            self.name,
            [self.parent_class],
            self.ctors,
            self.methods,
            self.static_methods,
            self.dunder_methods,
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

    def instantiate_parent_class(self, typenames):
        """
        Instantiate the inherited parent names.

        Args:
            typenames: List of template types to instantiate.

        Return: List of constructors instantiated with provided template args.
        """

        if isinstance(self.original.parent_class, parser.type.TemplatedType):
            return instantiate_type(
                self.original.parent_class, typenames, self.instantiations,
                parser.Typename(self.namespaces())).typename
        else:
            return self.original.parent_class

    def instantiate_ctors(self, typenames):
        """
        Instantiate the class constructors.

        Args:
            typenames: List of template types to instantiate.

        Return: List of constructors instantiated with provided template args.
        """

        helper = InstantiationHelper(
            instantiation_type=InstantiatedConstructor)

        instantiated_ctors = helper.multilevel_instantiation(
            self.original.ctors, typenames, self)

        return instantiated_ctors

    def instantiate_static_methods(self, typenames):
        """
        Instantiate static methods in the class.

        Args:
            typenames: List of template types to instantiate.

        Return: List of static methods instantiated with provided template args.
        """
        helper = InstantiationHelper(
            instantiation_type=InstantiatedStaticMethod)

        instantiated_static_methods = helper.multilevel_instantiation(
            self.original.static_methods, typenames, self)

        return instantiated_static_methods

    def instantiate_methods(self, typenames):
        """
        Instantiate regular methods in the class.

        Args:
            typenames: List of template types to instantiate.

        Return: List of methods instantiated with provided template args.
        """
        instantiated_methods = []

        helper = InstantiationHelper(instantiation_type=InstantiatedMethod)

        instantiated_methods = helper.multilevel_instantiation(
            self.original.methods, typenames, self)

        return instantiated_methods

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
        instantiated_ = instantiate_args_list(
            self.original.properties,
            typenames,
            self.instantiations,
            self.cpp_typename(),
        )
        # Convert to type Variable
        instantiated_properties = [
            parser.Variable(ctype=[arg.ctype],
                            name=arg.name,
                            default=arg.default) for arg in instantiated_
        ]
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
