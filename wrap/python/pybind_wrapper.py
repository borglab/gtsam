import argparse
import os

import interface_parser as parser

_use_boost = False
_ignore_classes = []


def gen_module_var(namespaces):
    idx = 1 if not namespaces[0] else 0
    return "m_{}".format('_'.join(namespaces[idx:]))


def add_namespaces(name, namespaces):
    if namespaces:
        # Ignore the first empty global namespace.
        idx = 1 if not namespaces[0] else 0
        return '::'.join(namespaces[idx:] + [name])
    else:
        return name


def qualifed_type_to_cpp(ctype):
    """
    Treat all pointers as "const shared_ptr<T>&"
    Treat Matrix and Vector as "const Matrix&" and "const Vector&" resp.
    """
    shared_ptr_ns = "boost" if _use_boost else "std"
    return "{const} {shared_ptr}{typename}{shared_ptr_ropbracket}{ref}".format(
        const="const" if ctype.is_const or ctype.is_ptr
        or ctype.typename.name in ["Matrix", "Vector"] else "",
        typename=ctype.typename,
        shared_ptr="{}::shared_ptr<".format(shared_ptr_ns)
        if ctype.is_ptr else "",
        shared_ptr_ropbracket=">" if ctype.is_ptr else "",
        ref="&" if ctype.is_ref or ctype.is_ptr
        or ctype.typename.name in ["Matrix", "Vector"] else "",
    )


def return_type_to_cpp(return_type):
    if return_type.type2:
        return "std::pair<{type1},{type2}>".format(
            type1=qualifed_type_to_cpp(return_type.type1),
            type2=qualifed_type_to_cpp(return_type.type2))
    else:
        return qualifed_type_to_cpp(return_type.type1)


def args_cpp_types(args_list):
    return [qualifed_type_to_cpp(arg.ctype) for arg in args_list]


def args_names(args_list):
    return [arg.name for arg in args_list]


def py_args_names(args_list):
    names = args_names(args_list)
    if names:
        py_args = ['py::arg("{}")'.format(name) for name in names]
        return ", " + ", ".join(py_args)
    else:
        return ''


def wrap_ctors(myclass):
    res = ""
    for ctor in myclass.ctors:
        res += '\n' + ' '*8 + '.def(py::init<{args_cpp_types}>()'\
            '{py_args_names})'.format(
                args_cpp_types=", ".join(args_cpp_types(ctor.args.args_list)),
                py_args_names=py_args_names(ctor.args.args_list),
            )
    return res


def method_args_signature(args_list):
    return ','.join(args_cpp_types(args_list))


def method_args_signature_with_names(args_list):
    cpp_types = args_cpp_types(args_list)
    names = args_names(args_list)
    types_names = [
        "{} {}".format(ctype, name) for ctype, name in zip(cpp_types, names)
    ]
    return ','.join(types_names)


def build_overloads(all_methods):
    overloads = {m.name: [] for m in all_methods}
    for method in all_methods:
        overloads[method.name].append(method)
    return overloads


def instantiate_args_list(args_list, template_typenames, instantiations,
                          cpp_class):
    """
    @param[in] args_list A list of `parser.Argument` to instantiate.
    @param[in] template_typenames List of template typenames to instantiate,
        e.g. ['T', 'U', 'V'].
    @param[in] instantiations List of specific types to instantiate, each
        associated with each template typename. Each type is a parser.Typename,
        including its name and full namespaces.
    @param[in] cpp_class Full-namespace cpp class name of this instantiation
        to replace for arguments of type `This`.
    @return A new list of parser.Argument which types are replaced with their
        instantiations.
    """
    new_args_list = []
    for arg in args_list:
        str_arg_typename = str(arg.ctype.typename)
        if str_arg_typename in template_typenames:
            idx = template_typenames.index(str_arg_typename)
            instantiation = instantiations[idx]
            new_type = parser.Type(
                typename=instantiation,
                is_const=arg.ctype.is_const,
                is_ptr=arg.ctype.is_ptr,
                is_ref=arg.ctype.is_ref,
                is_basis=arg.ctype.is_basis,
            )
            new_args_list.append(
                parser.Argument(name=arg.name, ctype=new_type))
        elif str_arg_typename == 'This':
            new_type = parser.Type(
                typename=cpp_class,
                is_const=arg.ctype.is_const,
                is_ptr=arg.ctype.is_ptr,
                is_ref=arg.ctype.is_ref,
                is_basis=arg.ctype.is_basis,
            )
            new_args_list.append(
                parser.Argument(ctype=new_type, name=arg.name))
        else:
            new_args_list.append(arg)
    return new_args_list


def wrap_method_lambda(
        py_method,
        cpp_class,
        cpp_method,
        return_void,
        has_overloads,
        args_signature_with_names,
        args_names,
        py_args_names,
        is_const,
        is_static,
        is_global,
        prefix,
        suffix,
):
    if cpp_method == "serialize" or cpp_method == "serializable":
        return ''
    is_method = not is_global and not is_static
    caller = cpp_class + "::" if not is_method else "self->"
    function_call = '{opt_return} {caller}{function_name}'\
        '({args_names});'.format(
            opt_return='return' if not return_void else '',
            caller=caller,
            function_name=cpp_method,
            args_names=', '.join(args_names),
        )
    return '{prefix}.{cdef}("{py_method}",'\
        '[]({opt_self}{opt_comma}{args_signature_with_names}){{'\
        '{function_call}'\
        '}}'\
        '{py_args_names}){suffix}'.format(
            prefix=prefix,
            cdef="def_static" if is_static else "def",
            py_method=py_method if py_method != "print" else "print_",
            opt_self="{cpp_class}* self".format(cpp_class=cpp_class)
            if is_method else "",
            cpp_class=cpp_class,
            cpp_method=cpp_method,
            opt_comma=',' if is_method and args_names else '',
            args_signature_with_names=args_signature_with_names,
            function_call=function_call,
            py_args_names=py_args_names,
            suffix=suffix,
        )


def wrap_methods(overloads, cpp_class, prefix='\n' + ' ' * 8, suffix=''):
    res = ""
    for name, methods in overloads.iteritems():
        has_overloads = (len(methods) > 1)
        for method in methods:
            is_method = isinstance(method, parser.Method)
            is_static = isinstance(method, parser.StaticMethod)
            is_global = isinstance(method, parser.GlobalFunction)
            if is_method and method.template:
                if len(method.template.typenames) > 1:
                    raise ValueError(
                        "Not yet handle multi-typenames template methods.")
                instantiations = method.template.instantiations[0]
                template_typename = method.template.typenames[0]
                cpp_methods = [
                    "{}<{}>".format(name, inst) for inst in instantiations
                ]
                # TODO(duy): To avoid conflicts, we should include the
                # instantiation's namespaces, but I find that too verbose.
                py_methods = [
                    "{}{}".format(name, inst.name) for inst in instantiations
                ]
                args_lists = [
                    instantiate_args_list(method.args.args_list,
                                          [template_typename], [inst],
                                          cpp_class) for inst in instantiations
                ]
            else:
                cpp_methods = [name]
                py_methods = [name]
                args_lists = [method.args.args_list]
            for cpp_method, py_method, args_list in zip(
                    cpp_methods, py_methods, args_lists):
                res += wrap_method_lambda(
                    py_method=py_method,
                    cpp_method=cpp_method,
                    cpp_class=cpp_class,
                    return_void=method.return_type.is_void(),
                    has_overloads=has_overloads,
                    args_signature_with_names=method_args_signature_with_names(
                        args_list),
                    args_names=args_names(args_list),
                    py_args_names=py_args_names(args_list),
                    is_const=is_method and method.is_const,
                    is_static=is_static,
                    is_global=is_global,
                    prefix=prefix,
                    suffix=suffix,
                )
    return res


def instantiate_class_template(myclass,
                               namespaces,
                               template_typenames,
                               corresponding_instantiations,
                               py_class_name=""):
    cpp_class = "{}<{}>".format(
        add_namespaces(myclass.name, namespaces),
        ", ".join([str(inst) for inst in corresponding_instantiations]))

    # TODO(duy): To avoid conflicts, we should include the instantiation's
    # namespaces, but I find that too verbose.
    new_class_name = py_class_name if py_class_name else "{}{}".format(
        myclass.name, "".join(
            [inst.name for inst in corresponding_instantiations]))
    new_ctors = []
    for ctor in myclass.ctors:
        new_args_list = instantiate_args_list(
            ctor.args.args_list, template_typenames,
            corresponding_instantiations, cpp_class)
        new_ctors.append(
            parser.Constructor(
                name=new_class_name, args=parser.ArgumentList(new_args_list)))
    new_methods = []
    for method in myclass.methods:
        new_args_list = instantiate_args_list(
            method.args.args_list, template_typenames,
            corresponding_instantiations, cpp_class)
        new_methods.append(
            parser.Method(
                template=method.template,
                name=method.name,
                return_type=method.return_type,  # don't care.
                args=parser.ArgumentList(new_args_list),
                is_const=method.is_const,
            ))
    new_static_methods = []
    for static_method in myclass.static_methods:
        new_args_list = instantiate_args_list(
            static_method.args.args_list, template_typenames,
            corresponding_instantiations, cpp_class)
        new_static_methods.append(
            parser.StaticMethod(
                name=static_method.name,
                return_type=static_method.return_type,  # don't care.
                args=parser.ArgumentList(new_args_list),
            ))

    return cpp_class, parser.Class(
        template='',
        is_virtual=myclass.is_virtual,
        name=new_class_name,
        parent=myclass.parent,
        methods_list=parser.MethodList(new_ctors + new_methods +
                                       new_static_methods),
    )


def instantiate_class_single_template(myclass, namespaces):
    if not myclass.template:
        raise ValueError("Not a template class.")
    if len(myclass.template.
           typenames) > 1 and myclass.template.instantiations[0]:
        raise ValueError("Can't instantiate multi-typenames template yet. "
                         "Please use typedef template instantiation.")

    template_typename = str(myclass.template.typenames[0])
    new_parser_classes = []
    cpp_classes = []
    for instantiation in myclass.template.instantiations[0]:
        cpp_class, new_parser_class = instantiate_class_template(
            myclass,
            namespaces,
            [template_typename],
            [instantiation],
        )
        cpp_classes.append(cpp_class)
        new_parser_classes.append(new_parser_class)
    return new_parser_classes, cpp_classes


def wrap_instantiated_class(new_class, cpp_class, namespaces):
    module_var = gen_module_var(namespaces)
    return '\n    py::class_<{cpp_class}, {class_parent}'\
        'std::shared_ptr<{cpp_class}>>({module_var}, "{class_name}")'\
        '{wrapped_ctors}'\
        '{wrapped_methods}'\
        '{wrapped_static_methods};\n'.format(
            cpp_class=cpp_class,
            class_name=new_class.name,
            class_parent=str(new_class.parent) +
            (', ' if new_class.parent else ''),
            module_var=module_var,
            wrapped_ctors=wrap_ctors(new_class),
            wrapped_methods=wrap_methods(
                build_overloads(new_class.methods),
                cpp_class,
            ),
            wrapped_static_methods=wrap_methods(
                build_overloads(new_class.static_methods),
                cpp_class,
            ),
        )


def wrap_class(my_class, namespaces):
    if my_class.template:
        classes, cpp_classes = instantiate_class_single_template(
            my_class, namespaces)
    else:
        classes = [my_class]
        cpp_classes = [add_namespaces(my_class.name, namespaces)]

    res = ''
    for new_class, cpp_class in zip(classes, cpp_classes):
        if cpp_class in _ignore_classes:
            continue
        res += wrap_instantiated_class(new_class, cpp_class, namespaces)
    return res


def wrap_typedef_template_instantiation(typedef_inst, module):
    myclass = module.find_class(typedef_inst.template_class)
    namespaces = typedef_inst.template_class.namespaces

    cpp_class, new_class = instantiate_class_template(
        myclass,
        namespaces,
        myclass.template.typenames,
        typedef_inst.instantiation,
        typedef_inst.new_class,
    )

    return wrap_instantiated_class(new_class, cpp_class, namespaces)


def wrap_namespace(namespace, module, prev_namespaces_list=[]):
    includes = ""
    wrapped = ""

    namespaces = prev_namespaces_list + [namespace.namespace]
    module_var = gen_module_var(namespaces)

    if namespace.namespace:
        wrapped += ' '*4 + 'pybind11::module {module_var} = '\
            '{parent_module_var}.def_submodule("{namespace}", "{namespace} '\
            'submodule");\n'.format(
                module_var=module_var,
                namespace=namespace.namespace,
                parent_module_var=gen_module_var(namespaces[:-1]),
            )

    for element in namespace.content:
        if isinstance(element, parser.Include):
            includes += "{}\n".format(element).replace('<', '"').replace(
                '>', '"')
        elif isinstance(element, parser.Namespace):
            wrapped_namespace, includes_namespace = wrap_namespace(
                element, module, namespaces)
            wrapped += wrapped_namespace
            includes += includes_namespace
        elif isinstance(element, parser.Class):
            wrapped += wrap_class(element, namespaces)
        elif isinstance(element, parser.TypedefTemplateInstantiation):
            wrapped += wrap_typedef_template_instantiation(element, module)

    # Global functions.
    all_funcs = [
        func for func in namespace.content
        if isinstance(func, parser.GlobalFunction)
    ]
    wrapped += wrap_methods(
        build_overloads(all_funcs),
        add_namespaces('', namespaces)[:-2],
        prefix='\n' + ' ' * 4 + module_var,
        suffix=';',
    )

    return wrapped, includes


def wrap_module(module_name, module, use_boost):
    wrapped_namespace, includes = wrap_namespace(module, module)

    return """
{include_boost}

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

{includes}

{hoder_type}

using namespace std;

namespace py = pybind11;

PYBIND11_PLUGIN({module_name}) {{
    pybind11::module m_("{module_name}", "pybind11 wrapper of {module_name}");

{wrapped_namespace}

    return m_.ptr();
}}

""".format(
        include_boost="#include <boost/shared_ptr.hpp>" if use_boost else "",
        module_name=module_name,
        includes=includes,
        hoder_type="PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>);"
        if use_boost else "",
        wrapped_namespace=wrapped_namespace)


def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument(
        "--src", type=str, required=True, help="Input interface .h file")
    arg_parser.add_argument("--module_name", type=str, required=True)
    arg_parser.add_argument(
        "--out", type=str, required=True, help="Output pybind .cc file")
    arg_parser.add_argument(
        "--use_boost",
        action="store_true",
        help="using boost's shared_ptr instead of std's")
    arg_parser.add_argument(
        "--ignore", nargs='*', type=str, help="Ignore classes")
    args = arg_parser.parse_args()

    global _use_boost
    global _ignore_classes
    _use_boost = args.use_boost
    _ignore_classes = args.ignore

    with open(args.src, "r") as f:
        content = f.read()
    module = parser.Module.parseString(content)
    cc_content = wrap_module(args.module_name, module, _use_boost)
    with open(args.out, "w") as f:
        f.write(cc_content)


if __name__ == "__main__":
    main()
