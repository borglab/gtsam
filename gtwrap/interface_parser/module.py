"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Rules and classes for parsing a module.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=unnecessary-lambda, unused-import, expression-not-assigned, no-else-return, protected-access, too-few-public-methods, too-many-arguments

import sys

import pyparsing  # type: ignore
from pyparsing import (ParserElement, ParseResults, ZeroOrMore,
                       cppStyleComment, stringEnd)

from .classes import Class
from .declaration import ForwardDeclaration, Include
from .function import GlobalFunction
from .namespace import Namespace
from .template import TypedefTemplateInstantiation


class Module:
    """
    Module is just a global namespace.

    E.g.
    ```
    namespace gtsam {
        ...
    }
    ```
    """

    rule = (
        ZeroOrMore(ForwardDeclaration.rule  #
                   ^ Include.rule  #
                   ^ Class.rule  #
                   ^ TypedefTemplateInstantiation.rule  #
                   ^ GlobalFunction.rule  #
                   ^ Namespace.rule  #
                   ).setParseAction(lambda t: Namespace('', t.asList())) +
        stringEnd)

    rule.ignore(cppStyleComment)

    @staticmethod
    def parseString(s: str) -> ParseResults:
        """Parse the source string and apply the rules."""
        return Module.rule.parseString(s)[0]
