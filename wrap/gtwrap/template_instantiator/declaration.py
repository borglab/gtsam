"""Instantiate a forward declaration."""

import gtwrap.interface_parser as parser
from gtwrap.template_instantiator.helpers import instantiate_name


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
