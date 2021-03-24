"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Define the parser rules and classes for various C++ types.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=unnecessary-lambda, expression-not-assigned

from typing import Iterable, Union

from pyparsing import Forward, Optional, Or, ParseResults, delimitedList

from .tokens import (BASIS_TYPES, CONST, IDENT, LOPBRACK, RAW_POINTER, REF,
                     ROPBRACK, SHARED_POINTER)


class Typename:
    """
    Generic type which can be either a basic type or a class type,
    similar to C++'s `typename` aka a qualified dependent type.
    Contains type name with full namespace and template arguments.

    E.g.
    ```
    gtsam::PinholeCamera<gtsam::Cal3S2>
    ```

    will give the name as `PinholeCamera`, namespace as `gtsam`,
    and template instantiations as `[gtsam::Cal3S2]`.

    Args:
        namespaces_and_name: A list representing the namespaces of the type
            with the type being the last element.
        instantiations: Template parameters to the type.
    """

    namespaces_name_rule = delimitedList(IDENT, "::")
    instantiation_name_rule = delimitedList(IDENT, "::")
    rule = Forward()
    rule << (
        namespaces_name_rule("namespaces_and_name")  #
        + Optional(
            (LOPBRACK + delimitedList(rule, ",")
             ("instantiations") + ROPBRACK))).setParseAction(
                 lambda t: Typename(t.namespaces_and_name, t.instantiations))

    def __init__(self,
                 namespaces_and_name: ParseResults,
                 instantiations: Union[tuple, list, str, ParseResults] = ()):
        self.name = namespaces_and_name[
            -1]  # the name is the last element in this list
        self.namespaces = namespaces_and_name[:-1]

        if instantiations:
            if isinstance(instantiations, Iterable):
                self.instantiations = instantiations  # type: ignore
            else:
                self.instantiations = instantiations.asList()
        else:
            self.instantiations = []

        if self.name in ["Matrix", "Vector"] and not self.namespaces:
            self.namespaces = ["gtsam"]

    @staticmethod
    def from_parse_result(parse_result: Union[str, list]):
        """Unpack the parsed result to get the Typename instance."""
        return parse_result[0]

    def __repr__(self) -> str:
        return self.to_cpp()

    def instantiated_name(self) -> str:
        """Get the instantiated name of the type."""
        res = self.name
        for instantiation in self.instantiations:
            res += instantiation.instantiated_name()
        return res

    def to_cpp(self) -> str:
        """Generate the C++ code for wrapping."""
        idx = 1 if self.namespaces and not self.namespaces[0] else 0
        if self.instantiations:
            cpp_name = self.name + "<{}>".format(", ".join(
                [inst.to_cpp() for inst in self.instantiations]))
        else:
            cpp_name = self.name
        return '{}{}{}'.format(
            "::".join(self.namespaces[idx:]),
            "::" if self.namespaces[idx:] else "",
            cpp_name,
        )

    def __eq__(self, other) -> bool:
        if isinstance(other, Typename):
            return str(self) == str(other)
        else:
            return False

    def __ne__(self, other) -> bool:
        res = self.__eq__(other)
        return not res


class QualifiedType:
    """Type with qualifiers, such as `const`."""

    rule = (
        Typename.rule("typename")  #
        + Optional(
            SHARED_POINTER("is_shared_ptr") | RAW_POINTER("is_ptr")
            | REF("is_ref"))).setParseAction(lambda t: QualifiedType(t))

    def __init__(self, t: ParseResults):
        self.typename = Typename.from_parse_result(t.typename)
        self.is_shared_ptr = t.is_shared_ptr
        self.is_ptr = t.is_ptr
        self.is_ref = t.is_ref


class BasisType:
    """
    Basis types are the built-in types in C++ such as double, int, char, etc.

    When using templates, the basis type will take on the same form as the template.

    E.g.
        ```
        template<T = {double}>
        void func(const T& x);
        ```

        will give

        ```
        m_.def("CoolFunctionDoubleDouble",[](const double& s) {
            return wrap_example::CoolFunction<double,double>(s);
        }, py::arg("s"));
        ```
    """

    rule = (
        Or(BASIS_TYPES)("typename")  #
        + Optional(
            SHARED_POINTER("is_shared_ptr") | RAW_POINTER("is_ptr")
            | REF("is_ref"))  #
    ).setParseAction(lambda t: BasisType(t))

    def __init__(self, t: ParseResults):
        self.typename = Typename([t.typename])
        self.is_ptr = t.is_ptr
        self.is_shared_ptr = t.is_shared_ptr
        self.is_ref = t.is_ref


class Type:
    """The type value that is parsed, e.g. void, string, size_t."""
    rule = (
        Optional(CONST("is_const"))  #
        + (BasisType.rule("basis") | QualifiedType.rule("qualified"))  # BR
    ).setParseAction(lambda t: Type.from_parse_result(t))

    def __init__(self, typename: Typename, is_const: str, is_shared_ptr: str,
                 is_ptr: str, is_ref: str, is_basis: bool):
        self.typename = typename
        self.is_const = is_const
        self.is_shared_ptr = is_shared_ptr
        self.is_ptr = is_ptr
        self.is_ref = is_ref
        self.is_basis = is_basis

    @staticmethod
    def from_parse_result(t: ParseResults):
        """Return the resulting Type from parsing the source."""
        if t.basis:
            return Type(
                typename=t.basis.typename,
                is_const=t.is_const,
                is_shared_ptr=t.basis.is_shared_ptr,
                is_ptr=t.basis.is_ptr,
                is_ref=t.basis.is_ref,
                is_basis=True,
            )
        elif t.qualified:
            return Type(
                typename=t.qualified.typename,
                is_const=t.is_const,
                is_shared_ptr=t.qualified.is_shared_ptr,
                is_ptr=t.qualified.is_ptr,
                is_ref=t.qualified.is_ref,
                is_basis=False,
            )
        else:
            raise ValueError("Parse result is not a Type")

    def __repr__(self) -> str:
        return "{self.typename} " \
            "{self.is_const}{self.is_shared_ptr}{self.is_ptr}{self.is_ref}".format(
            self=self)

    def to_cpp(self, use_boost: bool) -> str:
        """
        Generate the C++ code for wrapping.

        Treat all pointers as "const shared_ptr<T>&"
        Treat Matrix and Vector as "const Matrix&" and "const Vector&" resp.
        """
        shared_ptr_ns = "boost" if use_boost else "std"

        if self.is_shared_ptr:
            # always pass by reference: https://stackoverflow.com/a/8741626/1236990
            typename = "{ns}::shared_ptr<{typename}>&".format(
                ns=shared_ptr_ns, typename=self.typename.to_cpp())
        elif self.is_ptr:
            typename = "{typename}*".format(typename=self.typename.to_cpp())
        elif self.is_ref or self.typename.name in ["Matrix", "Vector"]:
            typename = typename = "{typename}&".format(
                typename=self.typename.to_cpp())
        else:
            typename = self.typename.to_cpp()

        return ("{const}{typename}".format(
            const="const " if
            (self.is_const
             or self.typename.name in ["Matrix", "Vector"]) else "",
            typename=typename))
