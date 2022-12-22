"""Instantiate global function."""

import gtwrap.interface_parser as parser
from gtwrap.template_instantiator.helpers import (instantiate_args_list,
                                                  instantiate_name,
                                                  instantiate_return_type)


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
                "::".join(inst.namespaces + [inst.instantiated_name()])
                for inst in self.instantiations
            ]
            ret = "{}<{}>".format(self.original.name,
                                  ",".join(instantiated_names))
        else:
            ret = self.original.name
        return ret

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedGlobalFunction, self).__repr__())
