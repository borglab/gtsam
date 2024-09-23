"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Define the parser rules and classes for various C++ types.

Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=unnecessary-lambda, expression-not-assigned

from typing import List, Sequence, Union

from pyparsing import ParseResults  # type: ignore
from pyparsing import Forward, Optional, Or, delimitedList

from .tokens import (BASIC_TYPES, CONST, IDENT, LOPBRACK, RAW_POINTER, REF,
                     ROPBRACK, SHARED_POINTER)


class Typename:
    """
    Class which holds a type's name, full namespace, and template arguments.

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
                 instantiations: Sequence[ParseResults] = ()):
        self.name = t[-1]  # the name is the last element in this list
        self.namespaces = t[:-1]

        # If the first namespace is empty string, just get rid of it.
        if self.namespaces and self.namespaces[0] == '':
            self.namespaces.pop(0)

        if instantiations:
            if isinstance(instantiations, Sequence):
                self.instantiations = instantiations  # type: ignore
            else:
                self.instantiations = instantiations.asList()
        else:
            self.instantiations = []

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
        if self.instantiations:
            cpp_name = self.name + "<{}>".format(", ".join(
                [inst.to_cpp() for inst in self.instantiations]))
        else:
            cpp_name = self.name
        return '{}{}{}'.format(
            "::".join(self.namespaces),
            "::" if self.namespaces else "",
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

    When using templates, the basic type will take on the same form as the template.

    E.g.
    ```
    template<T = {double}>
    void func(const T& x);
    ```

    will give

    ```
    m_.def("funcDouble",[](const double& x){
        ::func<double>(x);
    }, py::arg("x"));
    ```
    """

    rule = (Or(BASIC_TYPES)("typename")).setParseAction(lambda t: BasicType(t))

    def __init__(self, t: ParseResults):
        self.typename = Typename(t)


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
    Parsed datatype, can be either a fundamental/basic type or a custom datatype.
    E.g. void, double, size_t, Matrix.
    Think of this as a high-level type which encodes the typename and other
    characteristics of the type.

    The type can optionally be a raw pointer, shared pointer or reference.
    Can also be optionally qualified with a `const`, e.g. `const int`.
    """
    rule = (
        Optional(CONST("is_const"))  #
        + (BasicType.rule("basic") | CustomType.rule("qualified"))  # BR
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
        # If the type is a basic/fundamental c++ type (e.g int, bool)
        if t.basic:
            return Type(
                typename=t.basic.typename,
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

    def to_cpp(self) -> str:
        """
        Generate the C++ code for wrapping.

        Treat all pointers as "const shared_ptr<T>&"
        """

        if self.is_shared_ptr:
            typename = "std::shared_ptr<{typename}>".format(
                typename=self.typename.to_cpp())
        elif self.is_ptr:
            typename = "{typename}*".format(typename=self.typename.to_cpp())
        elif self.is_ref:
            typename = typename = "{typename}&".format(
                typename=self.typename.to_cpp())
        else:
            typename = self.typename.to_cpp()

        return ("{const}{typename}".format(
            const="const " if self.is_const else "", typename=typename))

    def get_typename(self):
        """Convenience method to get the typename of this type."""
        return self.typename.name


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

    def to_cpp(self):
        """
        Generate the C++ code for wrapping.
        """
        # Use Type.to_cpp to do the heavy lifting for the template parameters.
        template_args = ", ".join([t.to_cpp() for t in self.template_params])

        typename = "{typename}<{template_args}>".format(
            typename=self.typename.qualified_name(),
            template_args=template_args)

        if self.is_shared_ptr:
            typename = f"std::shared_ptr<{typename}>"
        elif self.is_ptr:
            typename = "{typename}*".format(typename=typename)
        elif self.is_ref:
            typename = typename = "{typename}&".format(typename=typename)
        else:
            pass

        return ("{const}{typename}".format(
            const="const " if self.is_const else "", typename=typename))
