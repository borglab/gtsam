"""Class constructor instantiator."""

# pylint: disable=unused-argument

from typing import Iterable, List

import gtwrap.interface_parser as parser


class InstantiatedConstructor(parser.Constructor):
    """
    Instantiate constructor with template parameters.

    E.g.
    class A {
        template<X, Y>
        A(X x, Y y);
    }
    """
    def __init__(self,
                 original: parser.Constructor,
                 instantiations: Iterable[parser.Typename] = ()):
        self.original = original
        self.instantiations = instantiations
        self.name = original.name
        self.args = original.args
        self.template = original.template
        self.parent = original.parent

        super().__init__(self.name,
                         self.args,
                         self.template,
                         parent=self.parent)

    @classmethod
    def construct(cls, original: parser.Constructor, typenames: List[str],
                  class_instantiations: List[parser.Typename],
                  method_instantiations: List[parser.Typename],
                  instantiated_args: List[parser.Argument],
                  parent: 'InstantiatedClass'):
        """Class method to construct object as required by InstantiationHelper."""
        method = parser.Constructor(
            name=parent.name,
            args=parser.ArgumentList(instantiated_args),
            template=original.template,
            parent=parent,
        )
        return InstantiatedConstructor(method,
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
