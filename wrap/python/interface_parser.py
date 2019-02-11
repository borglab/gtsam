import os
import pyparsing
from pyparsing import (alphas, alphanums, cppStyleComment, delimitedList,
                       empty, stringEnd, CharsNotIn, Keyword, Forward, Word,
                       Literal, OneOrMore, Optional, Or, Group, Suppress,
                       ZeroOrMore)
from pyparsing import ParseException, ParserElement
import sys

ParserElement.enablePackrat()

IDENT = Word(alphas + '_', alphanums + '_')
POINTER, REF = map(Literal, "*&")
LPAREN, RPAREN, LBRACE, RBRACE, COLON, SEMI_COLON = map(Suppress, "(){}:;")
LOPBRACK, ROPBRACK, COMMA, EQUAL = map(Suppress, "<>,=")
CONST, VIRTUAL, CLASS, STATIC, PAIR, TEMPLATE, TYPEDEF, INCLUDE = map(
    Keyword, [
        "const",
        "virtual",
        "class",
        "static",
        "pair",
        "template",
        "typedef",
        "#include",
    ])
NAMESPACE = Keyword("namespace")
BASIS_TYPES = map(Keyword, [
    "void",
    "bool",
    "unsigned char",
    "char",
    "int",
    "size_t",
    "double",
    "float",
    "string",
])


class Typename(object):
    """
    Type's name with full namespaces.
    """
    rule = delimitedList(IDENT, "::").setParseAction(lambda t: Typename(t))

    def __init__(self, namespaces_name):
        self.namespaces = namespaces_name[:-1]
        self.name = namespaces_name[-1]
        if self.name in ["Matrix", "Vector"] and not self.namespaces:
            self.namespaces = ["gtsam"]

    def __repr__(self):
        return '{}{}{}'.format("::".join(self.namespaces),
                               "::" if self.namespaces else "", self.name)

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


class QualifiedType(object):
    """
    Type with qualifiers.
    """
    rule = (
        Optional(CONST("is_const")) + Typename.rule("typename") +
        Optional(POINTER("is_ptr")
                 | REF("is_ref"))
    ).setParseAction(
        lambda t: QualifiedType(t.typename, t.is_const, t.is_ptr, t.is_ref))

    def __init__(self, typename, is_const, is_ptr, is_ref):
        self.typename = typename
        self.is_const = is_const
        self.is_ptr = is_ptr
        self.is_ref = is_ref


class BasisType(object):
    """
    Basis types don't have qualifiers and only allow copy-by-value.
    """
    rule = Or(BASIS_TYPES).setParseAction(lambda t: Typename(t))


class Type(object):
    rule = (
        BasisType.rule("basis") | QualifiedType.rule("qualified")  # BR
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
        return '{} {}{}{}'.format(self.typename, self.is_const, self.is_ptr,
                                  self.is_ref)


class Argument(object):
    rule = (Type.rule("ctype") +
            IDENT("name")).setParseAction(lambda t: Argument(t.ctype, t.name))

    def __init__(self, ctype, name):
        self.ctype = ctype
        self.name = name

    def __repr__(self):
        return '{} {}'.format(self.ctype.__repr__(), self.name)


class ArgumentList(object):
    rule = Optional(delimitedList(Argument.rule)("args_list")).setParseAction(
        lambda t: ArgumentList.from_parse_result(t.args_list))

    def __init__(self, args_list):
        self.args_list = args_list

    @staticmethod
    def from_parse_result(parse_result):
        if parse_result:
            return ArgumentList(parse_result.asList())
        else:
            return ArgumentList([])

    def __repr__(self):
        return self.args_list.__repr__()


class ReturnType(object):
    _pair = PAIR.suppress() + LOPBRACK + Type.rule(
        "type1") + COMMA + Type.rule("type2") + ROPBRACK
    rule = (  # BR
        _pair ^ Type.rule("type1")
    ).setParseAction(lambda t: ReturnType(t.type1, t.type2))

    def __init__(self, type1, type2):
        self.type1 = type1
        self.type2 = type2
        if self.type1.is_const or self.type1.is_ref or (
                self.type2 and (self.type2.is_const or self.type2.is_ref)):
            raise ValueError("Cannot deal with const/ref return type yet.")

    def is_void(self):
        return self.type1.typename.name == "void" and not self.type2

    def __repr__(self):
        return "{}{}".format(
            self.type1, (', ' + self.type2.__repr__()) if self.type2 else '')


class TemplateTypenameAndInstantiations(object):
    rule = (
        IDENT("typename") +
        Optional(EQUAL + LBRACE + delimitedList(Typename.rule)
                 ("instantiations") + RBRACE)
    ).setParseAction(
        lambda t: TemplateTypenameAndInstantiations(
            t.typename,
            t.instantiations,
        )
    )

    def __init__(self, typename, instantiations):
        self.typename = typename
        if instantiations:
            self.instantiations = instantiations.asList()
        else:
            self.instantiations = []


class Template(object):
    rule = (  # BR
        TEMPLATE + LOPBRACK + delimitedList(
            TemplateTypenameAndInstantiations.rule)
        ("typename_and_instantiations_list") + ROPBRACK  # BR
    ).setParseAction(
        lambda t: Template(t.typename_and_instantiations_list.asList()))

    def __init__(self, typename_and_instantiations_list):
        ti_list = typename_and_instantiations_list
        self.typenames = [ti.typename for ti in ti_list]
        self.instantiations = [ti.instantiations for ti in ti_list]


template = Template.rule.parseString(
    "template< T = {gtsam::Point3, gtsam::noiseModel::Gaussian}, U>")[0]


class Method(object):
    rule = (
        Optional(Template.rule("template")) + ReturnType.rule("return_type") +
        IDENT("name") + LPAREN + ArgumentList.rule("args_list") + RPAREN +
        Optional(CONST("is_const")) + SEMI_COLON  # BR
    ).setParseAction(
        lambda t: Method(
            t.template,
            t.name,
            t.return_type,
            t.args_list,
            t.is_const,
        )
    )

    def __init__(self, template, name, return_type, args, is_const):
        self.template = template
        self.name = name
        self.return_type = return_type
        self.args = args
        self.is_const = is_const

    def __repr__(self):
        return "Method: {} {} {}({}){}".format(self.template, self.return_type,
                                               self.name, self.args,
                                               self.is_const)


class StaticMethod(object):
    rule = (
        STATIC + ReturnType.rule("return_type") + IDENT("name") + LPAREN +
        ArgumentList.rule("args_list") + RPAREN + SEMI_COLON  # BR
    ).setParseAction(
        lambda t: StaticMethod(
            t.name,
            t.return_type,
            t.args_list,
        )
    )

    def __init__(self, name, return_type, args):
        self.name = name
        self.return_type = return_type
        self.args = args


class Constructor(object):
    rule = (
        IDENT("name") + LPAREN + ArgumentList.rule("args_list") + RPAREN +
        SEMI_COLON  # BR
    ).setParseAction(lambda t: Constructor(t.name, t.args_list))

    def __init__(self, name, args):
        self.name = name
        self.args = args


class MethodList(object):
    rule = ZeroOrMore(Constructor.rule ^ StaticMethod.rule ^ Method.rule
                      ).setParseAction(lambda t: MethodList(t.asList()))

    def __init__(self, methods_list):
        self.ctors = []
        self.methods = []
        self.static_methods = []
        for m in methods_list:
            if isinstance(m, Constructor):
                self.ctors.append(m)
            elif isinstance(m, Method):
                self.methods.append(m)
            elif isinstance(m, StaticMethod):
                self.static_methods.append(m)


class Class(object):
    _parent = COLON + Typename.rule("parent")
    rule = (
        Optional(Template.rule("template")) + Optional(
            VIRTUAL("is_virtual")) + CLASS + IDENT("name") + Optional(_parent)
        + LBRACE + MethodList.rule("methods_list") + RBRACE + SEMI_COLON  # BR
    ).setParseAction(lambda t: Class(
        t.template,
        t.is_virtual,
        t.name,
        t.parent,
        t.methods_list,
    ))

    def __init__(self, template, is_virtual, name, parent, methods_list):
        self.template = template
        self.is_virtual = is_virtual
        self.name = name
        self.parent = parent
        self.ctors = methods_list.ctors
        self.methods = methods_list.methods
        self.static_methods = methods_list.static_methods
        # Make sure ctors' names and class name are the same.
        for ctor in self.ctors:
            if ctor.name != self.name:
                raise ValueError("Error in constructor name! {} != {}".format(
                    ctor.name, self.name))


class TypedefTemplateInstantiation(object):
    rule = (
        TYPEDEF + Typename.rule("template_class") + LOPBRACK + delimitedList(
            Typename.rule)("instantiation") + ROPBRACK + IDENT("new_class") +
        SEMI_COLON
    ).setParseAction(
        lambda t: TypedefTemplateInstantiation(
            t.template_class,
            t.instantiation.asList(),
            t.new_class,
        )
    )

    def __init__(self, template_class, instantiation, new_class):
        self.template_class = template_class
        self.instantiation = instantiation
        self.new_class = new_class


class Include(object):
    rule = (INCLUDE + LOPBRACK + CharsNotIn('>')("header") +
            ROPBRACK).setParseAction(lambda t: Include(t.header))

    def __init__(self, header):
        self.header = header

    def __repr__(self):
        return "#include <{}>".format(self.header)


class ForwardDeclaration(object):
    rule = (
         Optional(VIRTUAL("is_virtual")) + CLASS + IDENT("name") +
         Optional(COLON + Typename.rule("parent")) + SEMI_COLON
    ).setParseAction(
        lambda t: ForwardDeclaration(
            t.is_virtual,
            t.name,
            t.parent,
        )
    )

    def __init__(self, is_virtual, name, parent):
        self.is_virtual = is_virtual
        self.name = name
        self.parent = parent

    def __repr__(self):
        return "ForwardDeclaration: {} {}({})".format(self.is_virtual,
                                                      self.name, self.parent)


class GlobalFunction(object):
    rule = (
        ReturnType.rule("return_type") + IDENT("name") + LPAREN +
        ArgumentList.rule("args_list") + RPAREN + SEMI_COLON
    ).setParseAction(
        lambda t: GlobalFunction(
            t.name,
            t.return_type,
            t.args_list,
        )
    )

    def __init__(self, name, return_type, args_list):
        self.name = name
        self.return_type = return_type
        self.args = args_list
        self.is_const = None

    def __repr__(self):
        return "GlobalFunction:  {}{}({})".format(self.return_type, self.name,
                                                  self.args)


def find_sub_namespace(namespace, str_namespaces):
    if not str_namespaces:
        return [namespace]

    sub_namespaces = (ns for ns in namespace.content
                      if isinstance(ns, Namespace))

    found_namespaces = [
        ns for ns in sub_namespaces if ns.namespace == str_namespaces[0]
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
        NAMESPACE + IDENT("namespace") + LBRACE +  # BR
        ZeroOrMore(ForwardDeclaration.rule ^ Include.rule ^ Class.rule ^
                   TypedefTemplateInstantiation.rule ^ GlobalFunction.rule ^
                   rule)("content")  # BR
        + RBRACE).setParseAction(lambda t: Namespace.from_parse_result(t))

    def __init__(self, namespace, content):
        self.namespace = namespace
        self.content = content

    @staticmethod
    def from_parse_result(t):
        if t.content:
            content = t.content.asList()
        else:
            content = []
        return Namespace(t.namespace, content)

    def find_class(self, typename):
        """
        Find the Class object given its typename.
        We have to traverse the tree of namespaces.
        """
        namespaces = find_sub_namespace(self, typename.namespaces)
        res = []
        for namespace in namespaces:
            classes = (c for c in namespace.content if isinstance(c, Class))
            res += [c for c in classes if c.name == typename.name]
        if not res:
            raise ValueError(
                "Cannot find class {} in module!".format(typename))
        elif len(res) > 1:
            raise ValueError(
                "Found more than one classes {} in module!".format(typename))
        else:
            return res[0]


class Module(object):
    """
    Module is just a global namespace.
    """
    rule = ZeroOrMore(ForwardDeclaration.rule ^ Include.rule ^ Class.rule ^
                      TypedefTemplateInstantiation.rule ^ GlobalFunction.rule ^
                      Namespace.rule).setParseAction(
                          lambda t: Namespace('', t.asList())) + stringEnd

    rule.ignore(cppStyleComment)

    @staticmethod
    def parseString(str):
        return Module.rule.parseString(str)[0]
