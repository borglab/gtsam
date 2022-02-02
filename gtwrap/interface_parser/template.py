"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Classes and rules for parsing C++ templates and typedefs for template instantiations.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

from typing import List

from pyparsing import Optional, ParseResults, delimitedList  # type: ignore

from .tokens import (EQUAL, IDENT, LBRACE, LOPBRACK, RBRACE, ROPBRACK,
                     SEMI_COLON, TEMPLATE, TYPEDEF)
from .type import TemplatedType, Typename


class Template:
    """
    Rule to parse templated values in the interface file.

    E.g.
    template<POSE>  // this is the Template.
    class Camera { ... };
    """
    class TypenameAndInstantiations:
        """
        Rule to parse the template parameters.

        template<typename POSE>  // POSE is the Instantiation.
        """
        rule = (
            IDENT("typename")  #
            + Optional(  #
                EQUAL  #
                + LBRACE  #
                + ((delimitedList(TemplatedType.rule ^ Typename.rule)
                    ("instantiations")))  #
                + RBRACE  #
            )).setParseAction(lambda t: Template.TypenameAndInstantiations(
                t.typename, t.instantiations))

        def __init__(self, typename: str, instantiations: ParseResults):
            self.typename = typename

            self.instantiations = []
            if instantiations:
                for inst in instantiations:
                    x = inst.typename if isinstance(inst,
                                                    TemplatedType) else inst
                    self.instantiations.append(x)

    rule = (  # BR
        TEMPLATE  #
        + LOPBRACK  #
        + delimitedList(TypenameAndInstantiations.rule)(
            "typename_and_instantiations_list")  #
        + ROPBRACK  # BR
    ).setParseAction(
        lambda t: Template(t.typename_and_instantiations_list.asList()))

    def __init__(
            self,
            typename_and_instantiations_list: List[TypenameAndInstantiations]):
        ti_list = typename_and_instantiations_list
        self.typenames = [ti.typename for ti in ti_list]
        self.instantiations = [ti.instantiations for ti in ti_list]

    def __repr__(self) -> str:
        return "<{0}>".format(", ".join(self.typenames))


class TypedefTemplateInstantiation:
    """
    Rule for parsing typedefs (with templates) within the interface file.

    E.g.
    ```
    typedef SuperComplexName<Arg1, Arg2, Arg3> EasierName;
    ```
    """
    rule = (TYPEDEF + TemplatedType.rule("templated_type") +
            IDENT("new_name") +
            SEMI_COLON).setParseAction(lambda t: TypedefTemplateInstantiation(
                t.templated_type[0], t.new_name))

    def __init__(self,
                 templated_type: TemplatedType,
                 new_name: str,
                 parent: str = ''):
        self.typename = templated_type.typename
        self.new_name = new_name
        self.parent = parent

    def __repr__(self):
        return "Typedef: {new_name} = {typename}".format(
            new_name=self.new_name, typename=self.typename)
