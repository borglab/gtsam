"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser class and rules for parsing C++ enums.

Author: Varun Agrawal
"""

from pyparsing import delimitedList  # type: ignore

from .tokens import ENUM, IDENT, LBRACE, RBRACE, SEMI_COLON
from .type import Typename
from .utils import collect_namespaces


class Enumerator:
    """
    Rule to parse an enumerator inside an enum.
    """
    rule = (
        IDENT("enumerator")).setParseAction(lambda t: Enumerator(t.enumerator))

    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return "Enumerator: ({0})".format(self.name)


class Enum:
    """
    Rule to parse enums defined in the interface file.

    E.g.
    ```
    enum Kind {
        Dog,
        Cat
    };
    ```
    """

    rule = (ENUM + IDENT("name") + LBRACE +
            delimitedList(Enumerator.rule)("enumerators") + RBRACE +
            SEMI_COLON).setParseAction(lambda t: Enum(t.name, t.enumerators))

    def __init__(self, name, enumerators, parent=''):
        self.name = name
        self.enumerators = enumerators
        self.parent = parent

    def namespaces(self) -> list:
        """Get the namespaces which this class is nested under as a list."""
        return collect_namespaces(self)

    def cpp_typename(self):
        """
        Return a Typename with the namespaces and cpp name of this
        class.
        """
        namespaces_name = self.namespaces()
        namespaces_name.append(self.name)
        return Typename(namespaces_name)

    def __repr__(self):
        return "Enum: {0}".format(self.name)
