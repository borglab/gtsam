"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser classes and rules for parsing C++ functions.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

from typing import Any, Iterable, List, Union

from pyparsing import Optional, ParseResults, delimitedList  # type: ignore

from .template import Template
from .tokens import (COMMA, DEFAULT_ARG, EQUAL, IDENT, LOPBRACK, LPAREN, PAIR,
                     ROPBRACK, RPAREN, SEMI_COLON)
from .type import TemplatedType, Type


class Argument:
    """
    The type and name of a function/method argument.

    E.g.
    ```
    void sayHello(/*`s` is the method argument with type `const string&`*/ const string& s);
    ```
    """
    rule = ((Type.rule ^ TemplatedType.rule)("ctype")  #
            + IDENT("name")  #
            + Optional(EQUAL + DEFAULT_ARG)("default")
            ).setParseAction(lambda t: Argument(
                t.ctype,  #
                t.name,  #
                t.default[0] if isinstance(t.default, ParseResults) else None))

    def __init__(self,
                 ctype: Union[Type, TemplatedType],
                 name: str,
                 default: ParseResults = None):
        if isinstance(ctype, Iterable):
            self.ctype = ctype[0]  # type: ignore
        else:
            self.ctype = ctype
        self.name = name
        self.default = default
        self.parent: Union[ArgumentList, None] = None

    def __repr__(self) -> str:
        return self.to_cpp()

    def to_cpp(self) -> str:
        """Return full C++ representation of argument."""
        return '{} {}'.format(repr(self.ctype), self.name)


class ArgumentList:
    """
    List of Argument objects for all arguments in a function.
    """
    rule = Optional(delimitedList(Argument.rule)("args_list")).setParseAction(
        lambda t: ArgumentList.from_parse_result(t.args_list))

    def __init__(self, args_list: List[Argument]):
        self.args_list = args_list
        for arg in args_list:
            arg.parent = self
        # The parent object which contains the argument list
        # E.g. Method, StaticMethod, Template, Constructor, GlobalFunction
        self.parent: Any = None

    @staticmethod
    def from_parse_result(parse_result: ParseResults):
        """Return the result of parsing."""
        if parse_result:
            return ArgumentList(parse_result.asList())
        else:
            return ArgumentList([])

    def __repr__(self) -> str:
        return repr(tuple(self.args_list))

    def __len__(self) -> int:
        return len(self.args_list)

    def names(self) -> List[str]:
        """Return a list of the names of all the arguments."""
        return [arg.name for arg in self.args_list]

    def list(self) -> List[Argument]:
        """Return a list of the names of all the arguments."""
        return self.args_list

    def to_cpp(self, use_boost: bool) -> List[str]:
        """Generate the C++ code for wrapping."""
        return [arg.ctype.to_cpp(use_boost) for arg in self.args_list]


class ReturnType:
    """
    Rule to parse the return type.

    The return type can either be a single type or a pair such as <type1, type2>.
    """
    _pair = (
        PAIR.suppress()  #
        + LOPBRACK  #
        + Type.rule("type1")  #
        + COMMA  #
        + Type.rule("type2")  #
        + ROPBRACK  #
    )
    rule = (_pair ^
            (Type.rule ^ TemplatedType.rule)("type1")).setParseAction(  # BR
                lambda t: ReturnType(t.type1, t.type2))

    def __init__(self, type1: Union[Type, TemplatedType], type2: Type):
        # If a TemplatedType, the return is a ParseResults, so we extract out the type.
        self.type1 = type1[0] if isinstance(type1, ParseResults) else type1
        self.type2 = type2
        # The parent object which contains the return type
        # E.g. Method, StaticMethod, Template, Constructor, GlobalFunction
        self.parent: Any = None

    def is_void(self) -> bool:
        """
        Check if the return type is void.
        """
        return self.type1.typename.name == "void" and not self.type2

    def __repr__(self) -> str:
        return "{}{}".format(
            self.type1, (', ' + self.type2.__repr__()) if self.type2 else '')

    def to_cpp(self, use_boost: bool) -> str:
        """
        Generate the C++ code for wrapping.

        If there are two return types, we return a pair<>,
        otherwise we return the regular return type.
        """
        if self.type2:
            return "std::pair<{type1},{type2}>".format(
                type1=self.type1.to_cpp(use_boost),
                type2=self.type2.to_cpp(use_boost))
        else:
            return self.type1.to_cpp(use_boost)


class GlobalFunction:
    """
    Rule to parse functions defined in the global scope.
    """
    rule = (
        Optional(Template.rule("template")) + ReturnType.rule("return_type")  #
        + IDENT("name")  #
        + LPAREN  #
        + ArgumentList.rule("args_list")  #
        + RPAREN  #
        + SEMI_COLON  #
    ).setParseAction(lambda t: GlobalFunction(t.name, t.return_type, t.
                                              args_list, t.template))

    def __init__(self,
                 name: str,
                 return_type: ReturnType,
                 args_list: ArgumentList,
                 template: Template,
                 parent: Any = ''):
        self.name = name
        self.return_type = return_type
        self.args = args_list
        self.template = template

        self.parent = parent
        self.return_type.parent = self
        self.args.parent = self

    def __repr__(self) -> str:
        return "GlobalFunction:  {}{}({})".format(self.return_type, self.name,
                                                  self.args)

    def to_cpp(self) -> str:
        """Generate the C++ code for wrapping."""
        return self.name
