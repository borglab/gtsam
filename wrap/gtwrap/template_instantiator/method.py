"""Class method and static method instantiators."""

from typing import Iterable

import gtwrap.interface_parser as parser
from gtwrap.template_instantiator.helpers import (instantiate_name,
                                                  instantiate_return_type)


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
        self.template = original.template
        self.is_const = original.is_const
        self.parent = original.parent

        self.name = instantiate_name(original.name, self.instantiations)
        self.return_type = original.return_type
        self.args = original.args

        super().__init__(self.template,
                         self.name,
                         self.return_type,
                         self.args,
                         self.is_const,
                         parent=self.parent)

    @classmethod
    def construct(cls, original, typenames, class_instantiations,
                  method_instantiations, instantiated_args, parent):
        """Class method to construct object as required by InstantiationHelper."""
        method = parser.Method(
            template=original.template,
            name=original.name,
            return_type=instantiate_return_type(
                original.return_type, typenames,
                class_instantiations + method_instantiations,
                parent.cpp_typename()),
            args=parser.ArgumentList(instantiated_args),
            is_const=original.is_const,
            parent=parent,
        )
        return InstantiatedMethod(method, instantiations=method_instantiations)

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
        return "Instantiated {}".format(super().__repr__())


class InstantiatedStaticMethod(parser.StaticMethod):
    """
    Instantiate static method with template parameters.
    """
    def __init__(self,
                 original: parser.StaticMethod,
                 instantiations: Iterable[parser.Typename] = ()):
        self.original = original
        self.instantiations = instantiations

        self.name = instantiate_name(original.name, self.instantiations)
        self.return_type = original.return_type
        self.args = original.args
        self.template = original.template
        self.parent = original.parent

        super().__init__(self.name, self.return_type, self.args, self.template,
                         self.parent)

    @classmethod
    def construct(cls, original, typenames, class_instantiations,
                  method_instantiations, instantiated_args, parent):
        """Class method to construct object as required by InstantiationHelper."""
        method = parser.StaticMethod(
            name=original.name,
            return_type=instantiate_return_type(original.return_type,
                                                typenames,
                                                class_instantiations +
                                                method_instantiations,
                                                parent.cpp_typename(),
                                                instantiated_class=parent),
            args=parser.ArgumentList(instantiated_args),
            template=original.template,
            parent=parent,
        )
        return InstantiatedStaticMethod(method,
                                        instantiations=method_instantiations)

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
        return "Instantiated {}".format(super().__repr__())
