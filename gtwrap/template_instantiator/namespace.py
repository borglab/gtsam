"""Instantiate a namespace."""

import itertools

import gtwrap.interface_parser as parser
from gtwrap.template_instantiator.classes import InstantiatedClass
from gtwrap.template_instantiator.declaration import InstantiatedDeclaration
from gtwrap.template_instantiator.function import InstantiatedGlobalFunction


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
