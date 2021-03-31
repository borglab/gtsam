"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Classes and rules for parsing C++ templates and typedefs for template instantiations.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

from typing import List

from pyparsing import Optional, ParseResults, delimitedList

from .tokens import (EQUAL, IDENT, LBRACE, LOPBRACK, RBRACE, ROPBRACK,
                     SEMI_COLON, TEMPLATE, TYPEDEF)
from .type import Typename


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
                + ((delimitedList(Typename.rule)("instantiations")))  #
                + RBRACE  #
            )).setParseAction(lambda t: Template.TypenameAndInstantiations(
                t.typename, t.instantiations))

        def __init__(self, typename: str, instantiations: ParseResults):
            self.typename = typename

            if instantiations:
                self.instantiations = instantiations.asList()
            else:
                self.instantiations = []

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
    rule = (TYPEDEF + Typename.rule("typename") + IDENT("new_name") +
            SEMI_COLON).setParseAction(lambda t: TypedefTemplateInstantiation(
                Typename.from_parse_result(t.typename), t.new_name))

    def __init__(self, typename: Typename, new_name: str, parent: str = ''):
        self.typename = typename
        self.new_name = new_name
        self.parent = parent
