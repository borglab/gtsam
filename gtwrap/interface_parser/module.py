"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Rules and classes for parsing a module.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=unnecessary-lambda, unused-import, expression-not-assigned, no-else-return, protected-access, too-few-public-methods, too-many-arguments

from pyparsing import (ParseBaseException, ZeroOrMore, cpp_style_comment,  # type: ignore
                       string_end)

from .annotations import (UNSUPPORTED_ANNOTATION,
                          UNSUPPORTED_TEMPLATED_ANNOTATION)
from .classes import Class
from .declaration import ForwardDeclaration, Include
from .enum import Enum
from .function import GlobalFunction
from .namespace import Namespace
from .template import TypedefTemplateInstantiation
from .variable import Variable


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
                   ^ Enum.rule  #
                   ^ Variable.rule  #
                   ^ Namespace.rule  #
                   ^ UNSUPPORTED_TEMPLATED_ANNOTATION  #
                   ^ UNSUPPORTED_ANNOTATION  #
                   ).set_parse_action(lambda t: Namespace('', t.as_list())) +
        string_end)

    rule.ignore(cpp_style_comment)

    @staticmethod
    def parse_string(s: str, source_name: str = "<string>") -> Namespace:
        """Parse source text and report any failure at its best known location."""
        # Imported here to avoid adding the diagnostic machinery to the grammar's
        # import cycle.
        from .diagnostics import (InterfaceParseError, begin_diagnostics,
                                  end_diagnostics)

        context, token = begin_diagnostics(s, source_name)
        try:
            return Module.rule.parse_string(s)[0]
        except InterfaceParseError:
            raise
        except ParseBaseException as error:
            diagnostic = InterfaceParseError.from_failure(context, error)
            raise diagnostic from error
        finally:
            end_diagnostics(token)
