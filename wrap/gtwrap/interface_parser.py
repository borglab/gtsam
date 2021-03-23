"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser to get the interface of a C++ source file
Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal, and Frank Dellaert
"""

# pylint: disable=unnecessary-lambda, unused-import, expression-not-assigned, no-else-return, protected-access, too-few-public-methods, too-many-arguments

import sys
from typing import Iterable, Union, Tuple, List

import pyparsing  # type: ignore
from pyparsing import (CharsNotIn, Forward, Group, Keyword, Literal, OneOrMore,
                       Optional, Or, ParseException, ParseResults, ParserElement, Suppress,
                       Word, ZeroOrMore, alphanums, alphas, cppStyleComment,
                       delimitedList, empty, nums, stringEnd)

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


ParserElement.enablePackrat()

# rule for identifiers (e.g. variable names)
IDENT = Word(alphas + '_', alphanums + '_') ^ Word(nums)

RAW_POINTER, SHARED_POINTER, REF = map(Literal, "@*&")

LPAREN, RPAREN, LBRACE, RBRACE, COLON, SEMI_COLON = map(Suppress, "(){}:;")
LOPBRACK, ROPBRACK, COMMA, EQUAL = map(Suppress, "<>,=")
CONST, VIRTUAL, CLASS, STATIC, PAIR, TEMPLATE, TYPEDEF, INCLUDE = map(
    Keyword,
    [
        "const",
        "virtual",
        "class",
        "static",
        "pair",
        "template",
        "typedef",
        "#include",
    ],
)
NAMESPACE = Keyword("namespace")
BASIS_TYPES = map(
    Keyword,
    [
        "void",
        "bool",
        "unsigned char",
        "char",
        "int",
        "size_t",
        "double",
        "float",
        "string",
    ],
)


class Typename:
    """Template typename list, with full namespaces."""

    namespaces_name_rule = delimitedList(IDENT, "::")
    instantiation_name_rule = delimitedList(IDENT, "::")
    rule = Forward()

    rule << (
        namespaces_name_rule("namespaces_name")  #
        + Optional(
            (LOPBRACK + delimitedList(rule, ",")("instantiations") + ROPBRACK))
    ).setParseAction(lambda t: Typename(t.namespaces_name, t.instantiations))

    def __init__(self,
                 namespaces_name: pyparsing.ParseResults,
                 instantiations: Union[tuple, list, str, ParseResults] = ()):
        self.namespaces = namespaces_name[:-1]
        self.name = namespaces_name[-1]

        if instantiations:
            if not isinstance(instantiations, Iterable):
                self.instantiations = instantiations.asList()
            else:
                self.instantiations = instantiations  # type: ignore
        else:
            self.instantiations = []

        if self.name in ["Matrix", "Vector"] and not self.namespaces:
            self.namespaces = ["gtsam"]

    @staticmethod
    def from_parse_result(parse_result: Union[str, list]):
        """Return the typename from the parsed result."""
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
            cpp_name = self.name + "<{}>".format(
                ", ".join([inst.to_cpp() for inst in self.instantiations])
            )
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
            return NotImplemented

    def __ne__(self, other) -> bool:
        res = self.__eq__(other)
        if res is NotImplemented:
            return res
        return not res


class Type:
    """The type value that is parsed, e.g. void, string, size_t."""
    class _QualifiedType:
        """Type with qualifiers, such as `const`."""

        rule = (
            Typename.rule("typename")  #
            + Optional(SHARED_POINTER("is_shared_ptr") | RAW_POINTER("is_ptr") | REF("is_ref"))
        ).setParseAction(
            lambda t: Type._QualifiedType(
                Typename.from_parse_result(t.typename),
                t.is_shared_ptr,
                t.is_ptr,
                t.is_ref,
            )
        )

        def __init__(self, typename: Typename, is_shared_ptr: str, is_ptr: str,
                     is_ref: str):
            self.typename = typename
            self.is_shared_ptr = is_shared_ptr
            self.is_ptr = is_ptr
            self.is_ref = is_ref

    class _BasisType:
        """
        Basis types are the built-in types in C++.

        They only allow copy-by-value, i.e. (double& x) is illegal,
        and cannot be used with smart pointers.

        The only exception is when using templates.
        If the template type is used as a const ref, the basis type will also be a const ref.

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
            + Optional(RAW_POINTER("is_ptr"))  #
        ).setParseAction(lambda t: Type._BasisType(
            # Set the first value in the list as a pseudo-namespace
            Typename(['', t.typename]),
            t.is_ptr))

        def __init__(self, typename: Typename, is_ptr: str):
            self.typename = typename
            self.is_ptr = is_ptr

    rule = (
        Optional(CONST("is_const"))  #
        + (_BasisType.rule("basis") | _QualifiedType.rule("qualified"))  # BR
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
                is_shared_ptr='',
                is_ptr=t.basis.is_ptr,
                is_ref='',
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
            raise ValueError("Parse result is not a Type?")

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


class Argument:
    """
    The type and name of a function/method argument.

    E.g.
    ```
    void sayHello(/*`s` is the method argument with type `const string&`*/ const string& s);
    ```
    """
    rule = (Type.rule("ctype") +
            IDENT("name")).setParseAction(lambda t: Argument(t.ctype, t.name))

    def __init__(self, ctype: Type, name: str):
        self.ctype = ctype
        self.name = name
        self.parent: Union[ArgumentList, None] = None

    def __repr__(self) -> str:
        return '{} {}'.format(self.ctype.__repr__(), self.name)


class ArgumentList:
    """
    List of Argument objects for all arguments in a function.
    """
    rule = Optional(delimitedList(Argument.rule)("args_list")).setParseAction(
        lambda t: ArgumentList.from_parse_result(t.args_list)
    )

    def __init__(self, args_list: List[Argument]):
        self.args_list = args_list
        for arg in args_list:
            arg.parent = self
        self.parent: Union[Method, StaticMethod, Template, Constructor,
                           GlobalFunction, None] = None

    @staticmethod
    def from_parse_result(parse_result: ParseResults):
        """Return the result of parsing."""
        if parse_result:
            return ArgumentList(parse_result.asList())
        else:
            return ArgumentList([])

    def __repr__(self) -> str:
        return self.args_list.__repr__()

    def args_names(self) -> List[str]:
        """Return a list of the names of all the arguments."""
        return [arg.name for arg in self.args_list]

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
    rule = (_pair ^ Type.rule("type1")).setParseAction(  # BR
        lambda t: ReturnType(t.type1, t.type2))

    def __init__(self, type1: Type, type2: Type):
        self.type1 = type1
        self.type2 = type2
        self.parent: Union[Method, StaticMethod, GlobalFunction, None] = None

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


class Template:
    """
    Rule to parse templated values in the interface file.

    E.g.
    template<POSE>  // this is the Template.
    class Camera { ... };
    """
    class TypenameAndInstantiations:
        """
        Rule to parse the template parameters.

        template<typename POSE>  // POSE is the Instantiation.
        """
        rule = (
            IDENT("typename")  #
            + Optional(  #
                EQUAL  #
                + LBRACE  #
                + ((delimitedList(Typename.rule)("instantiations")))  #
                + RBRACE  #
            )).setParseAction(lambda t: Template.TypenameAndInstantiations(
                t.typename, t.instantiations))

        def __init__(self, typename: str, instantiations: ParseResults):
            self.typename = typename

            if instantiations:
                self.instantiations = instantiations.asList()
            else:
                self.instantiations = []

    rule = (  # BR
        TEMPLATE  #
        + LOPBRACK  #
        + delimitedList(TypenameAndInstantiations.rule)(
            "typename_and_instantiations_list")  #
        + ROPBRACK  # BR
    ).setParseAction(
        lambda t: Template(t.typename_and_instantiations_list.asList()))

    def __init__(self, typename_and_instantiations_list: List[TypenameAndInstantiations]):
        ti_list = typename_and_instantiations_list
        self.typenames = [ti.typename for ti in ti_list]
        self.instantiations = [ti.instantiations for ti in ti_list]

    def __repr__(self) -> str:
        return "<{0}>".format(", ".join(self.typenames))


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

    def __init__(self, name: str, args: ArgumentList, parent: Union["Class", str] =''):
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
        Type.rule("ctype")  #
        + IDENT("name")  #
        + SEMI_COLON  #
    ).setParseAction(lambda t: Property(t.ctype, t.name))

    def __init__(self, ctype: Type, name: str, parent=''):
        self.ctype = ctype
        self.name = name
        self.parent = parent

    def __repr__(self) -> str:
        return '{} {}'.format(self.ctype.__repr__(), self.name)


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
    class MethodsAndProperties:
        """
        Rule for all the methods and properties within a class.
        """
        rule = ZeroOrMore(
            Constructor.rule ^ StaticMethod.rule ^ Method.rule ^ Property.rule
        ).setParseAction(lambda t: Class.MethodsAndProperties(t.asList()))

        def __init__(self, methods_props: List[Union[Constructor, Method,
                                                StaticMethod, Property]]):
            self.ctors = []
            self.methods = []
            self.static_methods = []
            self.properties = []
            for m in methods_props:
                if isinstance(m, Constructor):
                    self.ctors.append(m)
                elif isinstance(m, Method):
                    self.methods.append(m)
                elif isinstance(m, StaticMethod):
                    self.static_methods.append(m)
                elif isinstance(m, Property):
                    self.properties.append(m)

    _parent = COLON + Typename.rule("parent_class")
    rule = (
        Optional(Template.rule("template"))  #
        + Optional(VIRTUAL("is_virtual"))  #
        + CLASS  #
        + IDENT("name")  #
        + Optional(_parent)  #
        + LBRACE  #
        + MethodsAndProperties.rule("methods_props")  #
        + RBRACE  #
        + SEMI_COLON  # BR
    ).setParseAction(lambda t: Class(
        t.template,
        t.is_virtual,
        t.name,
        t.parent_class,
        t.methods_props.ctors,
        t.methods_props.methods,
        t.methods_props.static_methods,
        t.methods_props.properties,
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
        parent: str = '',
    ):
        self.template = template
        self.is_virtual = is_virtual
        self.name = name
        if parent_class:
            self.parent_class = Typename.from_parse_result(parent_class)
        else:
            self.parent_class = ''

        self.ctors = ctors
        self.methods = methods
        self.static_methods = static_methods
        self.properties = properties
        self.parent = parent
        # Make sure ctors' names and class name are the same.
        for ctor in self.ctors:
            if ctor.name != self.name:
                raise ValueError(
                    "Error in constructor name! {} != {}".format(
                        ctor.name, self.name
                    )
                )

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


class TypedefTemplateInstantiation:
    """
    Rule for parsing typedefs (with templates) within the interface file.

    E.g.
    ```
    typedef SuperComplexName<Arg1, Arg2, Arg3> EasierName;
    ```
    """
    rule = (
        TYPEDEF + Typename.rule("typename") + IDENT("new_name") + SEMI_COLON
    ).setParseAction(
        lambda t: TypedefTemplateInstantiation(
            Typename.from_parse_result(t.typename), t.new_name
        )
    )

    def __init__(self, typename: Typename, new_name: str, parent: str=''):
        self.typename = typename
        self.new_name = new_name
        self.parent = parent


class Include:
    """
    Rule to parse #include directives.
    """
    rule = (
        INCLUDE + LOPBRACK + CharsNotIn('>')("header") + ROPBRACK
    ).setParseAction(lambda t: Include(t.header))

    def __init__(self, header: CharsNotIn, parent: str = ''):
        self.header = header
        self.parent = parent

    def __repr__(self) -> str:
        return "#include <{}>".format(self.header)


class ForwardDeclaration:
    """
    Rule to parse forward declarations in the interface file.
    """
    rule = (
        Optional(VIRTUAL("is_virtual"))
        + CLASS
        + Typename.rule("name")
        + Optional(COLON + Typename.rule("parent_type"))
        + SEMI_COLON
    ).setParseAction(
        lambda t: ForwardDeclaration(t.name, t.parent_type, t.is_virtual)
    )

    def __init__(self,
                 name: Typename,
                 parent_type: str,
                 is_virtual: str,
                 parent: str = ''):
        self.name = name
        if parent_type:
            self.parent_type = Typename.from_parse_result(parent_type)
        else:
            self.parent_type = ''

        self.is_virtual = is_virtual
        self.parent = parent

    def __repr__(self) -> str:
        return "ForwardDeclaration: {} {}({})".format(self.is_virtual,
                                                      self.name, self.parent)


class GlobalFunction:
    """
    Rule to parse functions defined in the global scope.
    """
    rule = (
        Optional(Template.rule("template"))
        + ReturnType.rule("return_type")  #
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
                 parent: str = ''):
        self.name = name
        self.return_type = return_type
        self.args = args_list
        self.template = template

        self.parent = parent
        self.return_type.parent = self
        self.args.parent = self

    def __repr__(self) -> str:
        return "GlobalFunction:  {}{}({})".format(
            self.return_type, self.name, self.args
        )

    def to_cpp(self) -> str:
        """Generate the C++ code for wrapping."""
        return self.name


def find_sub_namespace(namespace: "Namespace",
                       str_namespaces: List["Namespace"]) -> list:
    """
    Get the namespaces nested under `namespace`, filtered by a list of namespace strings.

    Args:
        namespace: The top-level namespace under which to find sub-namespaces.
        str_namespaces: The list of namespace strings to filter against.
    """
    if not str_namespaces:
        return [namespace]

    sub_namespaces = (
        ns for ns in namespace.content if isinstance(ns, Namespace)
    )

    found_namespaces = [
        ns for ns in sub_namespaces if ns.name == str_namespaces[0]
    ]
    if not found_namespaces:
        return []

    res = []
    for found_namespace in found_namespaces:
        ns = find_sub_namespace(found_namespace, str_namespaces[1:])
        if ns:
            res += ns
    return res


class Namespace:
    """Rule for parsing a namespace in the interface file."""

    rule = Forward()
    rule << (
        NAMESPACE  #
        + IDENT("name")  #
        + LBRACE  #
        + ZeroOrMore(  # BR
            ForwardDeclaration.rule  #
            ^ Include.rule  #
            ^ Class.rule  #
            ^ TypedefTemplateInstantiation.rule  #
            ^ GlobalFunction.rule  #
            ^ rule  #
        )("content")  # BR
        + RBRACE  #
    ).setParseAction(lambda t: Namespace.from_parse_result(t))

    def __init__(self, name: str, content: ZeroOrMore, parent=''):
        self.name = name
        self.content = content
        self.parent = parent
        for child in self.content:
            child.parent = self

    @staticmethod
    def from_parse_result(t: ParseResults):
        """Return the result of parsing."""
        if t.content:
            content = t.content.asList()
        else:
            content = []
        return Namespace(t.name, content)

    def find_class_or_function(
            self, typename: Typename) -> Union[Class, GlobalFunction]:
        """
        Find the Class or GlobalFunction object given its typename.
        We have to traverse the tree of namespaces.
        """
        found_namespaces = find_sub_namespace(self, typename.namespaces)
        res = []
        for namespace in found_namespaces:
            classes_and_funcs = (c for c in namespace.content if isinstance(c, (Class, GlobalFunction)))
            res += [c for c in classes_and_funcs if c.name == typename.name]
        if not res:
            raise ValueError(
                "Cannot find class {} in module!".format(typename.name)
            )
        elif len(res) > 1:
            raise ValueError(
                "Found more than one classes {} in module!".format(
                    typename.name
                )
            )
        else:
            return res[0]

    def top_level(self) -> "Namespace":
        """Return the top leve namespace."""
        if self.name == '' or self.parent == '':
            return self
        else:
            return self.parent.top_level()

    def __repr__(self) -> str:
        return "Namespace: {}\n\t{}".format(self.name, self.content)

    def full_namespaces(self) -> List["Namespace"]:
        """Get the full namespace list."""
        ancestors = collect_namespaces(self)
        if self.name:
            ancestors.append(self.name)
        return ancestors


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
