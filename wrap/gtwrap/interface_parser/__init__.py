"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser to get the interface of a C++ source file

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

import sys

import pyparsing  # type: ignore

from .classes import *
from .declaration import *
from .enum import *
from .function import *
from .module import *
from .namespace import *
from .template import *
from .tokens import *
from .type import *
from .variable import *
from .diagnostics import InterfaceParseError, track_rule as _track_rule

# Fix deepcopy issue with pyparsing
# Can remove once https://github.com/pyparsing/pyparsing/issues/208 is resolved.
if sys.version_info >= (3, 8):

    def fixed_get_attr(self, item):
        """
        Fix for monkey-patching issue with deepcopy in pyparsing.ParseResults
        """
        if item == '__deepcopy__':
            raise AttributeError(item)
        try:
            return self[item]
        except KeyError:
            return ""

    # apply the monkey-patch
    pyparsing.ParseResults.__getattr__ = fixed_get_attr

_DIAGNOSTIC_RULES = (
    (Type.rule, "type", 10),
    (TemplatedType.rule, "templated type", 20),
    (Argument.rule, "argument", 100),
    (ArgumentList.rule, "argument list", 90),
    (ReturnType.rule, "return type", 30),
    (Method.rule, "method declaration", 80),
    (StaticMethod.rule, "static method declaration", 80),
    (Constructor.rule, "constructor declaration", 80),
    (Operator.rule, "operator declaration", 80),
    (DunderMethod.rule, "dunder method declaration", 80),
    (Variable.rule, "variable declaration", 70),
    (Enumerator.rule, "enumerator", 100),
    (Enum.rule, "enum declaration", 70),
    (Template.rule, "template declaration", 60),
    (TypedefTemplateInstantiation.rule, "typedef declaration", 60),
    (ForwardDeclaration.rule, "forward declaration", 60),
    (Include.rule, "include declaration", 60),
    (Class.rule, "class declaration", 50),
    (Namespace.rule, "namespace declaration", 40),
    (GlobalFunction.rule, "global function declaration", 50),
)

for _rule, _context, _priority in _DIAGNOSTIC_RULES:
    _track_rule(_rule, _context, _priority)

pyparsing.ParserElement.enable_packrat()
