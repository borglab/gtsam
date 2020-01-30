"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Parser to get the interface of a C++ source file
Author: Duy Nguyen Ta, Fan Jiang, Matthew Sklar and Frank Dellaert
"""
import os
import sys

from pyparsing import (
    alphas,
    alphanums,
    cppStyleComment,
    delimitedList,
    empty,
    nums,
    stringEnd,
    CharsNotIn,
    Forward,
    Group,
    Keyword,
    Literal,
    OneOrMore,
    Optional,
    Or,
    ParseException,
    ParserElement,
    Suppress,
    Word,
    ZeroOrMore,
)

ParserElement.enablePackrat()

IDENT = Word(alphas + '_', alphanums + '_') ^ Word(nums)
POINTER, REF = map(Literal, "*&")
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


class Typename(object):
    """
    Type's name with full namespaces.
    """

    namespaces_name_rule = delimitedList(IDENT, "::")
    instantiation_name_rule = delimitedList(IDENT, "::")
    rule = Forward()

    rule << (
        namespaces_name_rule("namespaces_name")
        + Optional(
            (LOPBRACK + delimitedList(rule, ",")("instantiations") + ROPBRACK)
        )
    ).setParseAction(lambda t: Typename(t.namespaces_name, t.instantiations))

    def __init__(self, namespaces_name, instantiations=''):
        self.namespaces = namespaces_name[:-1]
        self.name = namespaces_name[-1]

        if instantiations:
            self.instantiations = instantiations.asList()
        else:
            self.instantiations = ''
        if self.name in ["Matrix", "Vector"] and not self.namespaces:
            self.namespaces = ["gtsam"]

    @staticmethod
    def from_parse_result(parse_result):
        return parse_result[0]

    def __repr__(self):
        return self.to_cpp()

    def instantiated_name(self):
        res = self.name
        for instantiation in self.instantiations:
            res += instantiation.instantiated_name()
        return res

    def to_cpp(self):
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

    def __eq__(self, other):
        if isinstance(other, Typename):
            return str(self) == str(other)
        else:
            return NotImplemented

    def __ne__(self, other):
        res = self.__eq__(other)
        if res is NotImplemented:
            return res
        return not res


class Type(object):
    class _QualifiedType(object):
        """
        Type with qualifiers.
        """

        rule = (
            Optional(CONST("is_const"))
            + Typename.rule("typename")
            + Optional(POINTER("is_ptr") | REF("is_ref"))
        ).setParseAction(
            lambda t: Type._QualifiedType(
                Typename.from_parse_result(t.typename),
                t.is_const,
                t.is_ptr,
                t.is_ref,
            )
        )

        def __init__(self, typename, is_const, is_ptr, is_ref):
            self.typename = typename
            self.is_const = is_const
            self.is_ptr = is_ptr
            self.is_ref = is_ref

    class _BasisType(object):
        """
        Basis types don't have qualifiers and only allow copy-by-value.
        """

        rule = Or(BASIS_TYPES).setParseAction(lambda t: Typename(t))

    rule = (
        _BasisType.rule("basis") | _QualifiedType.rule("qualified")  # BR
    ).setParseAction(lambda t: Type.from_parse_result(t))

    def __init__(self, typename, is_const, is_ptr, is_ref, is_basis):
        self.typename = typename
        self.is_const = is_const
        self.is_ptr = is_ptr
        self.is_ref = is_ref
        self.is_basis = is_basis

    @staticmethod
    def from_parse_result(t):
        if t.basis:
            return Type(
                typename=t.basis,
                is_const='',
                is_ptr='',
                is_ref='',
                is_basis=True,
            )
        elif t.qualified:
            return Type(
                typename=t.qualified.typename,
                is_const=t.qualified.is_const,
                is_ptr=t.qualified.is_ptr,
                is_ref=t.qualified.is_ref,
                is_basis=False,
            )
        else:
            raise ValueError("Parse result is not a Type?")

    def __repr__(self):
        return '{} {}{}{}'.format(
            self.typename, self.is_const, self.is_ptr, self.is_ref
        )

    def to_cpp(self, use_boost):
        """
        Treat all pointers as "const shared_ptr<T>&"
        Treat Matrix and Vector as "const Matrix&" and "const Vector&" resp.
        """
        shared_ptr_ns = "boost" if use_boost else "std"
        return (
            "{const} {shared_ptr}{typename}"
            "{shared_ptr_ropbracket}{ref}".format(
                const="const"
                if self.is_const
                or self.is_ptr
                or self.typename.name in ["Matrix", "Vector"]
                else "",
                typename=self.typename.to_cpp(),
                shared_ptr="{}::shared_ptr<".format(shared_ptr_ns)
                if self.is_ptr
                else "",
                shared_ptr_ropbracket=">" if self.is_ptr else "",
                ref="&"
                if self.is_ref
                or self.is_ptr
                or self.typename.name in ["Matrix", "Vector"]
                else "",
            )
        )


class Argument(object):
    rule = (Type.rule("ctype") + IDENT("name")).setParseAction(
        lambda t: Argument(t.ctype, t.name)
    )

    def __init__(self, ctype, name):
        self.ctype = ctype
        self.name = name

    def __repr__(self):
        return '{} {}'.format(self.ctype.__repr__(), self.name)


class ArgumentList(object):
    rule = Optional(delimitedList(Argument.rule)("args_list")).setParseAction(
        lambda t: ArgumentList.from_parse_result(t.args_list)
    )

    def __init__(self, args_list):
        self.args_list = args_list
        for arg in args_list:
            arg.parent = self

    @staticmethod
    def from_parse_result(parse_result):
        if parse_result:
            return ArgumentList(parse_result.asList())
        else:
            return ArgumentList([])

    def __repr__(self):
        return self.args_list.__repr__()

    def args_names(self):
        return [arg.name for arg in self.args_list]

    def to_cpp(self, use_boost):
        return [arg.ctype.to_cpp(use_boost) for arg in self.args_list]


class ReturnType(object):
    _pair = (
        PAIR.suppress()
        + LOPBRACK
        + Type.rule("type1")
        + COMMA
        + Type.rule("type2")
        + ROPBRACK
    )
    rule = (_pair ^ Type.rule("type1")).setParseAction(  # BR
        lambda t: ReturnType(t.type1, t.type2)
    )

    def __init__(self, type1, type2):
        self.type1 = type1
        self.type2 = type2

    def is_void(self):
        return self.type1.typename.name == "void" and not self.type2

    def __repr__(self):
        return "{}{}".format(
            self.type1, (', ' + self.type2.__repr__()) if self.type2 else ''
        )

    def to_cpp(self):
        if self.type2:
            return "std::pair<{type1},{type2}>".format(
                type1=self.type1.to_cpp(), type2=self.type2.to_cpp()
            )
        else:
            return self.type1.to_cpp()


class Template(object):
    class TypenameAndInstantiations(object):
        rule = (
            IDENT("typename")
            + Optional(
                EQUAL
                + LBRACE
                + ((delimitedList(Typename.rule)("instantiations")))
                + RBRACE
            )
        ).setParseAction(
            lambda t: Template.TypenameAndInstantiations(
                t.typename, t.instantiations
            )
        )

        def __init__(self, typename, instantiations):
            self.typename = typename

            if instantiations:
                self.instantiations = instantiations.asList()
            else:
                self.instantiations = []

    rule = (  # BR
        TEMPLATE
        + LOPBRACK
        + delimitedList(TypenameAndInstantiations.rule)(
            "typename_and_instantiations_list"
        )
        + ROPBRACK  # BR
    ).setParseAction(
        lambda t: Template(t.typename_and_instantiations_list.asList())
    )

    def __init__(self, typename_and_instantiations_list):
        ti_list = typename_and_instantiations_list
        self.typenames = [ti.typename for ti in ti_list]
        self.instantiations = [ti.instantiations for ti in ti_list]


class Method(object):
    rule = (
        Optional(Template.rule("template"))
        + ReturnType.rule("return_type")
        + IDENT("name")
        + LPAREN
        + ArgumentList.rule("args_list")
        + RPAREN
        + Optional(CONST("is_const"))
        + SEMI_COLON  # BR
    ).setParseAction(
        lambda t: Method(
            t.template, t.name, t.return_type, t.args_list, t.is_const
        )
    )

    def __init__(self, template, name, return_type, args, is_const, parent=''):
        self.template = template
        self.name = name
        self.return_type = return_type
        self.args = args
        self.is_const = is_const

        self.parent = parent

    def __repr__(self):
        return "Method: {} {} {}({}){}".format(
            self.template,
            self.return_type,
            self.name,
            self.args,
            self.is_const,
        )


class StaticMethod(object):
    rule = (
        STATIC
        + ReturnType.rule("return_type")
        + IDENT("name")
        + LPAREN
        + ArgumentList.rule("args_list")
        + RPAREN
        + SEMI_COLON  # BR
    ).setParseAction(
        lambda t: StaticMethod(t.name, t.return_type, t.args_list)
    )

    def __init__(self, name, return_type, args, parent=''):
        self.name = name
        self.return_type = return_type
        self.args = args

        self.parent = parent

    def __repr__(self):
        return "static {} {}{}".format(self.return_type, self.name, self.args)

    def to_cpp(self):
        return self.name


class Constructor(object):
    rule = (
        IDENT("name")
        + LPAREN
        + ArgumentList.rule("args_list")
        + RPAREN
        + SEMI_COLON  # BR
    ).setParseAction(lambda t: Constructor(t.name, t.args_list))

    def __init__(self, name, args, parent=''):
        self.name = name
        self.args = args

        self.parent = parent

    def __repr__(self):
        return "Constructor: {}".format(self.name)


class Property(object):
    rule = (Type.rule("ctype") + IDENT("name") + SEMI_COLON).setParseAction(
        lambda t: Property(t.ctype, t.name)
    )

    def __init__(self, ctype, name, parent=''):
        self.ctype = ctype
        self.name = name
        # Check type constraints: no pointer, no ref.
        if self.ctype.is_ptr or self.ctype.is_ref:
            raise ValueError("Can't deal with pointer/ref class properties.")

        self.parent = parent

    def __repr__(self):
        return '{} {}'.format(self.ctype.__repr__(), self.name)


def collect_namespaces(obj):
    namespaces = []
    ancestor = obj.parent
    while ancestor and ancestor.name:
        namespaces = [ancestor.name] + namespaces
        ancestor = ancestor.parent
    return [''] + namespaces


class Class(object):
    class MethodsAndProperties(object):
        rule = ZeroOrMore(
            Constructor.rule ^ StaticMethod.rule ^ Method.rule ^ Property.rule
        ).setParseAction(lambda t: Class.MethodsAndProperties(t.asList()))

        def __init__(self, methods_props):
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
        Optional(Template.rule("template"))
        + Optional(VIRTUAL("is_virtual"))
        + CLASS
        + IDENT("name")
        + Optional(_parent)
        + LBRACE
        + MethodsAndProperties.rule("methods_props")
        + RBRACE
        + SEMI_COLON  # BR
    ).setParseAction(
        lambda t: Class(
            t.template,
            t.is_virtual,
            t.name,
            t.parent_class,
            t.methods_props.ctors,
            t.methods_props.methods,
            t.methods_props.static_methods,
            t.methods_props.properties,
        )
    )

    def __init__(
        self,
        template,
        is_virtual,
        name,
        parent_class,
        ctors,
        methods,
        static_methods,
        properties,
        parent='',
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

    def namespaces(self):
        return collect_namespaces(self)


class TypedefTemplateInstantiation(object):
    rule = (
        TYPEDEF + Typename.rule("typename") + IDENT("new_name") + SEMI_COLON
    ).setParseAction(
        lambda t: TypedefTemplateInstantiation(
            Typename.from_parse_result(t.typename), t.new_name
        )
    )

    def __init__(self, typename, new_name, parent=''):
        self.typename = typename
        self.new_name = new_name
        self.parent = parent


class Include(object):
    rule = (
        INCLUDE + LOPBRACK + CharsNotIn('>')("header") + ROPBRACK
    ).setParseAction(lambda t: Include(t.header))

    def __init__(self, header, parent=''):
        self.header = header
        self.parent = parent

    def __repr__(self):
        return "#include <{}>".format(self.header)


class ForwardDeclaration(object):
    rule = (
        Optional(VIRTUAL("is_virtual"))
        + CLASS
        + Typename.rule("name")
        + Optional(COLON + Typename.rule("parent_type"))
        + SEMI_COLON
    ).setParseAction(
        lambda t: ForwardDeclaration(t.is_virtual, t.name, t.parent_type)
    )

    def __init__(self, is_virtual, name, parent_type, parent=''):
        self.is_virtual = is_virtual
        self.name = name
        if parent_type:
            self.parent_type = Typename.from_parse_result(parent_type)
        else:
            self.parent_type = ''
        self.parent = parent

    def __repr__(self):
        return "ForwardDeclaration: {} {}({})".format(
            self.is_virtual, self.name, self.parent
        )


class GlobalFunction(object):
    rule = (
        ReturnType.rule("return_type")
        + IDENT("name")
        + LPAREN
        + ArgumentList.rule("args_list")
        + RPAREN
        + SEMI_COLON
    ).setParseAction(
        lambda t: GlobalFunction(t.name, t.return_type, t.args_list)
    )

    def __init__(self, name, return_type, args_list, parent=''):
        self.name = name
        self.return_type = return_type
        self.args = args_list
        self.is_const = None

        self.parent = parent
        self.return_type.parent = self
        self.args.parent = self

    def __repr__(self):
        return "GlobalFunction:  {}{}({})".format(
            self.return_type, self.name, self.args
        )

    def to_cpp(self):
        return self.name


def find_sub_namespace(namespace, str_namespaces):
    if not str_namespaces:
        return [namespace]

    sub_namespaces = (
        ns for ns in namespace.content if isinstance(ns, Namespace)
    )

    found_namespaces = [
        ns for ns in sub_namespaces if ns.name == str_namespaces[0]
    ]
    if not found_namespaces:
        return None

    res = []
    for found_namespace in found_namespaces:
        ns = find_sub_namespace(found_namespace, str_namespaces[1:])
        if ns:
            res += ns
    return res


class Namespace(object):
    rule = Forward()
    rule << (
        NAMESPACE
        + IDENT("name")
        + LBRACE
        + ZeroOrMore(  # BR
            ForwardDeclaration.rule
            ^ Include.rule
            ^ Class.rule
            ^ TypedefTemplateInstantiation.rule
            ^ GlobalFunction.rule
            ^ rule
        )(
            "content"
        )  # BR
        + RBRACE
    ).setParseAction(lambda t: Namespace.from_parse_result(t))

    def __init__(self, name, content, parent=''):
        self.name = name
        self.content = content
        self.parent = parent
        for child in self.content:
            child.parent = self

    @staticmethod
    def from_parse_result(t):
        if t.content:
            content = t.content.asList()
        else:
            content = []
        return Namespace(t.name, content)

    def find_class(self, typename):
        """
        Find the Class object given its typename.
        We have to traverse the tree of namespaces.
        """
        found_namespaces = find_sub_namespace(self, typename.namespaces)
        res = []
        for namespace in found_namespaces:
            classes = (c for c in namespace.content if isinstance(c, Class))
            res += [c for c in classes if c.name == typename.name]
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

    def top_level(self):
        if self.name == '' or self.parent == '':
            return self
        else:
            return self.parent.top_level()

    def __repr__(self):
        return "Namespace: {}\n\t{}".format(self.name, self.content)

    def full_namespaces(self):
        ancestors = collect_namespaces(self)
        if self.name:
            ancestors.append(self.name)
        return ancestors


class Module(object):
    """
    Module is just a global namespace.
    """

    rule = (
        ZeroOrMore(
            ForwardDeclaration.rule
            ^ Include.rule
            ^ Class.rule
            ^ TypedefTemplateInstantiation.rule
            ^ GlobalFunction.rule
            ^ Namespace.rule
        ).setParseAction(lambda t: Namespace('', t.asList()))
        + stringEnd
    )

    rule.ignore(cppStyleComment)

    @staticmethod
    def parseString(str):
        return Module.rule.parseString(str)[0]
