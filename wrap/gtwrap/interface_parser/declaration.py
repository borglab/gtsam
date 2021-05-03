"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Classes and rules for declarations such as includes and forward declarations.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

from pyparsing import CharsNotIn, Optional

from .tokens import (CLASS, COLON, INCLUDE, LOPBRACK, ROPBRACK, SEMI_COLON,
                     VIRTUAL)
from .type import Typename


class Include:
    """
    Rule to parse #include directives.
    """
    rule = (INCLUDE + LOPBRACK + CharsNotIn('>')("header") +
            ROPBRACK).setParseAction(lambda t: Include(t.header))

    def __init__(self, header: CharsNotIn, parent: str = ''):
        self.header = header
        self.parent = parent

    def __repr__(self) -> str:
        return "#include <{}>".format(self.header)


class ForwardDeclaration:
    """
    Rule to parse forward declarations in the interface file.
    """
    rule = (Optional(VIRTUAL("is_virtual")) + CLASS + Typename.rule("name") +
            Optional(COLON + Typename.rule("parent_type")) +
            SEMI_COLON).setParseAction(lambda t: ForwardDeclaration(
                t.name, t.parent_type, t.is_virtual))

    def __init__(self,
                 name: Typename,
                 parent_type: str,
                 is_virtual: str,
                 parent: str = ''):
        self.name = name
        if parent_type:
            self.parent_type = parent_type
        else:
            self.parent_type = ''

        self.is_virtual = is_virtual
        self.parent = parent

    def __repr__(self) -> str:
        return "ForwardDeclaration: {} {}({})".format(self.is_virtual,
                                                      self.name, self.parent)
