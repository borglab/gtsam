"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Define the parser rules and classes for various C++ types.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=unnecessary-lambda, expression-not-assigned

from typing import Iterable, List, Union

from pyparsing import (Forward, Optional, Or, ParseResults,  # type: ignore
                       delimitedList)

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
    rule = (
        namespaces_name_rule("namespaces_and_name")  #
    ).setParseAction(lambda t: Typename(t))

    def __init__(self,
                 t: ParseResults,
                 instantiations: Iterable[ParseResults] = ()):
        self.name = t[-1]  # the name is the last element in this list
        self.namespaces = t[:-1]

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

    def qualified_name(self):
        """Return the fully qualified name, e.g. `gtsam::internal::PoseKey`."""
        return "::".join(self.namespaces + [self.name])

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


class BasicType:
    """
    Basic types are the fundamental built-in types in C++ such as double, int, char, etc.

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

    rule = (Or(BASIS_TYPES)("typename")).setParseAction(lambda t: BasicType(t))

    def __init__(self, t: ParseResults):
        self.typename = Typename(t.asList())


class CustomType:
    """
    Custom defined types with the namespace.
    Essentially any C++ data type that is not a BasicType.

    E.g.
    ```
    gtsam::Matrix wTc;
    ```

    Here `gtsam::Matrix` is a custom type.
    """

    rule = (Typename.rule("typename")).setParseAction(lambda t: CustomType(t))

    def __init__(self, t: ParseResults):
        self.typename = Typename(t)


class Type:
    """
    Parsed datatype, can be either a fundamental type or a custom datatype.
    E.g. void, double, size_t, Matrix.

    The type can optionally be a raw pointer, shared pointer or reference.
    Can also be optionally qualified with a `const`, e.g. `const int`.
    """
    rule = (
        Optional(CONST("is_const"))  #
        + (BasicType.rule("basis") | CustomType.rule("qualified"))  # BR
        + Optional(
            SHARED_POINTER("is_shared_ptr") | RAW_POINTER("is_ptr")
            | REF("is_ref"))  #
    ).setParseAction(lambda t: Type.from_parse_result(t))

    def __init__(self, typename: Typename, is_const: str, is_shared_ptr: str,
                 is_ptr: str, is_ref: str, is_basic: bool):
        self.typename = typename
        self.is_const = is_const
        self.is_shared_ptr = is_shared_ptr
        self.is_ptr = is_ptr
        self.is_ref = is_ref
        self.is_basic = is_basic

    @staticmethod
    def from_parse_result(t: ParseResults):
        """Return the resulting Type from parsing the source."""
        if t.basis:
            return Type(
                typename=t.basis.typename,
                is_const=t.is_const,
                is_shared_ptr=t.is_shared_ptr,
                is_ptr=t.is_ptr,
                is_ref=t.is_ref,
                is_basic=True,
            )
        elif t.qualified:
            return Type(
                typename=t.qualified.typename,
                is_const=t.is_const,
                is_shared_ptr=t.is_shared_ptr,
                is_ptr=t.is_ptr,
                is_ref=t.is_ref,
                is_basic=False,
            )
        else:
            raise ValueError("Parse result is not a Type")

    def __repr__(self) -> str:
        is_ptr_or_ref = "{0}{1}{2}".format(self.is_shared_ptr, self.is_ptr,
                                           self.is_ref)
        return "{is_const}{self.typename}{is_ptr_or_ref}".format(
            self=self,
            is_const="const " if self.is_const else "",
            is_ptr_or_ref=" " + is_ptr_or_ref if is_ptr_or_ref else "")

    def to_cpp(self, use_boost: bool) -> str:
        """
        Generate the C++ code for wrapping.

        Treat all pointers as "const shared_ptr<T>&"
        Treat Matrix and Vector as "const Matrix&" and "const Vector&" resp.

        Args:
            use_boost: Flag indicating whether to use boost::shared_ptr or std::shared_ptr.
        """
        shared_ptr_ns = "boost" if use_boost else "std"

        if self.is_shared_ptr:
            typename = "{ns}::shared_ptr<{typename}>".format(
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


class TemplatedType:
    """
    Parser rule for data types which are templated.
    This is done so that the template parameters can be pointers/references.

    E.g. std::vector<double>, BearingRange<Pose3, Point3>
    """

    rule = Forward()
    rule << (
        Optional(CONST("is_const"))  #
        + Typename.rule("typename")  #
        + (
            LOPBRACK  #
            + delimitedList(Type.rule ^ rule, ",")("template_params")  #
            + ROPBRACK)  #
        + Optional(
            SHARED_POINTER("is_shared_ptr") | RAW_POINTER("is_ptr")
            | REF("is_ref"))  #
    ).setParseAction(lambda t: TemplatedType.from_parse_result(t))

    def __init__(self, typename: Typename, template_params: List[Type],
                 is_const: str, is_shared_ptr: str, is_ptr: str, is_ref: str):
        instantiations = [param.typename for param in template_params]
        # Recreate the typename but with the template params as instantiations.
        self.typename = Typename(typename.namespaces + [typename.name],
                                 instantiations)

        self.template_params = template_params

        self.is_const = is_const
        self.is_shared_ptr = is_shared_ptr
        self.is_ptr = is_ptr
        self.is_ref = is_ref

    @staticmethod
    def from_parse_result(t: ParseResults):
        """Get the TemplatedType from the parser results."""
        return TemplatedType(t.typename, t.template_params, t.is_const,
                             t.is_shared_ptr, t.is_ptr, t.is_ref)

    def __repr__(self):
        return "TemplatedType({typename.namespaces}::{typename.name})".format(
            typename=self.typename)

    def to_cpp(self, use_boost: bool):
        """
        Generate the C++ code for wrapping.

        Args:
            use_boost: Flag indicating whether to use boost::shared_ptr or std::shared_ptr.
        """
        # Use Type.to_cpp to do the heavy lifting for the template parameters.
        template_args = ", ".join(
            [t.to_cpp(use_boost) for t in self.template_params])

        typename = "{typename}<{template_args}>".format(
            typename=self.typename.qualified_name(),
            template_args=template_args)

        shared_ptr_ns = "boost" if use_boost else "std"
        if self.is_shared_ptr:
            typename = "{ns}::shared_ptr<{typename}>".format(ns=shared_ptr_ns,
                                                             typename=typename)
        elif self.is_ptr:
            typename = "{typename}*".format(typename=typename)
        elif self.is_ref or self.typename.name in ["Matrix", "Vector"]:
            typename = typename = "{typename}&".format(typename=typename)
        else:
            pass

        return ("{const}{typename}".format(
            const="const " if
            (self.is_const
             or self.typename.name in ["Matrix", "Vector"]) else "",
            typename=typename))
