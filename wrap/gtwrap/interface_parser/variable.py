"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser classes and rules for parsing C++ variables.

Author: Varun Agrawal, Gerry Chen
"""

from typing import List

from pyparsing import Optional, ParseResults  # type: ignore

from .tokens import DEFAULT_ARG, EQUAL, IDENT, SEMI_COLON
from .type import TemplatedType, Type


class Variable:
    """
    Rule to parse variables.
    Variables are a combination of Type/TemplatedType and the variable identifier.

    E.g.
    ```
    class Hello {
        string name;  // This is a property variable.
    };

    Vector3 kGravity;  // This is a global variable.
    ````
    """
    rule = ((Type.rule ^ TemplatedType.rule)("ctype")  #
            + IDENT("name")  #
            + Optional(EQUAL + DEFAULT_ARG)("default")  #
            + SEMI_COLON  #
            ).setParseAction(lambda t: Variable(
                t.ctype,  #
                t.name,  #
                t.default[0] if isinstance(t.default, ParseResults) else None))

    def __init__(self,
                 ctype: List[Type],
                 name: str,
                 default: ParseResults = None,
                 parent=''):
        self.ctype = ctype[0]  # ParseResult is a list
        self.name = name
        self.default = default
        self.parent = parent

    def __repr__(self) -> str:
        return '{} {}'.format(self.ctype.__repr__(), self.name)
