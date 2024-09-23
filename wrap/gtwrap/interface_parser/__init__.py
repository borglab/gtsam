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

pyparsing.ParserElement.enablePackrat()
