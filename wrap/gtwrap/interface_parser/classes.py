"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser classes and rules for parsing C++ classes.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

from typing import Iterable, List, Union

from pyparsing import Optional, ZeroOrMore, Literal

from .function import ArgumentList, ReturnType
from .template import Template
from .tokens import (CLASS, COLON, CONST, IDENT, LBRACE, LPAREN, RBRACE,
                     RPAREN, SEMI_COLON, STATIC, VIRTUAL, OPERATOR)
from .type import TemplatedType, Type, Typename


class Method:
    """
    Rule to parse a method in a class.

    E.g.
    ```
    class Hello {
        void sayHello() const;
    };
    ```
    """
    rule = (
        Optional(Template.rule("template"))  #
        + ReturnType.rule("return_type")  #
        + IDENT("name")  #
        + LPAREN  #
        + ArgumentList.rule("args_list")  #
        + RPAREN  #
        + Optional(CONST("is_const"))  #
        + SEMI_COLON  # BR
    ).setParseAction(lambda t: Method(t.template, t.name, t.return_type, t.
                                      args_list, t.is_const))

    def __init__(self,
                 template: str,
                 name: str,
                 return_type: ReturnType,
                 args: ArgumentList,
                 is_const: str,
                 parent: Union[str, "Class"] = ''):
        self.template = template
        self.name = name
        self.return_type = return_type
        self.args = args
        self.is_const = is_const

        self.parent = parent

    def __repr__(self) -> str:
        return "Method: {} {} {}({}){}".format(
            self.template,
            self.return_type,
            self.name,
            self.args,
            self.is_const,
        )


class StaticMethod:
    """
    Rule to parse all the static methods in a class.

    E.g.
    ```
    class Hello {
        static void changeGreeting();
    };
    ```
    """
    rule = (
        STATIC  #
        + ReturnType.rule("return_type")  #
        + IDENT("name")  #
        + LPAREN  #
        + ArgumentList.rule("args_list")  #
        + RPAREN  #
        + SEMI_COLON  # BR
    ).setParseAction(
        lambda t: StaticMethod(t.name, t.return_type, t.args_list))

    def __init__(self,
                 name: str,
                 return_type: ReturnType,
                 args: ArgumentList,
                 parent: Union[str, "Class"] = ''):
        self.name = name
        self.return_type = return_type
        self.args = args

        self.parent = parent

    def __repr__(self) -> str:
        return "static {} {}{}".format(self.return_type, self.name, self.args)

    def to_cpp(self) -> str:
        """Generate the C++ code for wrapping."""
        return self.name


class Constructor:
    """
    Rule to parse the class constructor.
    Can have 0 or more arguments.
    """
    rule = (
        IDENT("name")  #
        + LPAREN  #
        + ArgumentList.rule("args_list")  #
        + RPAREN  #
        + SEMI_COLON  # BR
    ).setParseAction(lambda t: Constructor(t.name, t.args_list))

    def __init__(self,
                 name: str,
                 args: ArgumentList,
                 parent: Union["Class", str] = ''):
        self.name = name
        self.args = args

        self.parent = parent

    def __repr__(self) -> str:
        return "Constructor: {}".format(self.name)


class Property:
    """
    Rule to parse the variable members of a class.

    E.g.
    ```
    class Hello {
        string name;  // This is a property.
    };
    ````
    """
    rule = (
        (Type.rule ^ TemplatedType.rule)("ctype")  #
        + IDENT("name")  #
        + SEMI_COLON  #
    ).setParseAction(lambda t: Property(t.ctype, t.name))

    def __init__(self, ctype: Type, name: str, parent=''):
        self.ctype = ctype[0]  # ParseResult is a list
        self.name = name
        self.parent = parent

    def __repr__(self) -> str:
        return '{} {}'.format(self.ctype.__repr__(), self.name)


class Operator:
    """
    Rule for parsing operator overloads.

    E.g.
    ```
    class Overload {
        Vector2 operator+(const Vector2 &v) const;
    };
    """
    rule = (
        ReturnType.rule("return_type")  #
        + Literal("operator")("name")  #
        + OPERATOR("operator")  #
        + LPAREN  #
        + ArgumentList.rule("args_list")  #
        + RPAREN  #
        + CONST("is_const")  #
        + SEMI_COLON  # BR
    ).setParseAction(lambda t: Operator(t.name, t.operator, t.return_type, t.
                                        args_list, t.is_const))

    def __init__(self,
                 name: str,
                 operator: str,
                 return_type: ReturnType,
                 args: ArgumentList,
                 is_const: str,
                 parent: Union[str, "Class"] = ''):
        self.name = name
        self.operator = operator
        self.return_type = return_type
        self.args = args
        self.is_const = is_const
        self.is_unary = len(args) == 0

        self.parent = parent

        # Check for valid unary operators
        if self.is_unary and self.operator not in ('+', '-'):
            raise ValueError("Invalid unary operator {} used for {}".format(
                self.operator, self))

        # Check that number of arguments are either 0 or 1
        assert 0 <= len(args) < 2, \
            "Operator overload should be at most 1 argument, " \
                "{} arguments provided".format(len(args))

        # Check to ensure arg and return type are the same.
        if len(args) == 1 and self.operator not in ("()", "[]"):
            assert args.args_list[0].ctype.typename.name == return_type.type1.typename.name, \
                "Mixed type overloading not supported. Both arg and return type must be the same."

    def __repr__(self) -> str:
        return "Operator: {}{}{}({}) {}".format(
            self.return_type,
            self.name,
            self.operator,
            self.args,
            self.is_const,
        )


def collect_namespaces(obj):
    """
    Get the chain of namespaces from the lowest to highest for the given object.

    Args:
        obj: Object of type Namespace, Class or InstantiatedClass.
    """
    namespaces = []
    ancestor = obj.parent
    while ancestor and ancestor.name:
        namespaces = [ancestor.name] + namespaces
        ancestor = ancestor.parent
    return [''] + namespaces


class Class:
    """
    Rule to parse a class defined in the interface file.

    E.g.
    ```
    class Hello {
        ...
    };
    ```
    """
    class Members:
        """
        Rule for all the members within a class.
        """
        rule = ZeroOrMore(Constructor.rule ^ StaticMethod.rule ^ Method.rule
                          ^ Property.rule ^ Operator.rule).setParseAction(
                              lambda t: Class.Members(t.asList()))

        def __init__(self,
                     members: List[Union[Constructor, Method, StaticMethod,
                                         Property, Operator]]):
            self.ctors = []
            self.methods = []
            self.static_methods = []
            self.properties = []
            self.operators = []
            for m in members:
                if isinstance(m, Constructor):
                    self.ctors.append(m)
                elif isinstance(m, Method):
                    self.methods.append(m)
                elif isinstance(m, StaticMethod):
                    self.static_methods.append(m)
                elif isinstance(m, Property):
                    self.properties.append(m)
                elif isinstance(m, Operator):
                    self.operators.append(m)

    _parent = COLON + Typename.rule("parent_class")
    rule = (
        Optional(Template.rule("template"))  #
        + Optional(VIRTUAL("is_virtual"))  #
        + CLASS  #
        + IDENT("name")  #
        + Optional(_parent)  #
        + LBRACE  #
        + Members.rule("members")  #
        + RBRACE  #
        + SEMI_COLON  # BR
    ).setParseAction(lambda t: Class(
        t.template,
        t.is_virtual,
        t.name,
        t.parent_class,
        t.members.ctors,
        t.members.methods,
        t.members.static_methods,
        t.members.properties,
        t.members.operators,
    ))

    def __init__(
        self,
        template: Template,
        is_virtual: str,
        name: str,
        parent_class: list,
        ctors: List[Constructor],
        methods: List[Method],
        static_methods: List[StaticMethod],
        properties: List[Property],
        operators: List[Operator],
        parent: str = '',
    ):
        self.template = template
        self.is_virtual = is_virtual
        self.name = name
        if parent_class:
            if isinstance(parent_class, Iterable):
                self.parent_class = parent_class[0]
            else:
                self.parent_class = parent_class

        else:
            self.parent_class = ''

        self.ctors = ctors
        self.methods = methods
        self.static_methods = static_methods
        self.properties = properties
        self.operators = operators
        self.parent = parent

        # Make sure ctors' names and class name are the same.
        for ctor in self.ctors:
            if ctor.name != self.name:
                raise ValueError("Error in constructor name! {} != {}".format(
                    ctor.name, self.name))

        for ctor in self.ctors:
            ctor.parent = self
        for method in self.methods:
            method.parent = self
        for static_method in self.static_methods:
            static_method.parent = self
        for _property in self.properties:
            _property.parent = self

    def namespaces(self) -> list:
        """Get the namespaces which this class is nested under as a list."""
        return collect_namespaces(self)

    def __repr__(self):
        return "Class: {self.name}".format(self=self)
