import interface_parser as parser


def instantiate_type(ctype, template_typenames, instantiations, cpp_typename, instantiated_class=None):
    """
    Instantiate template typename for @p ctype.
    @return If ctype's name is in the @p template_typenames, return the
        corresponding type to replace in @p instantiations.
        If ctype name is `This`, return the new typename @p `cpp_typename`.
        Otherwise, return the original ctype.
    """
    str_arg_typename = str(ctype.typename)
    if str_arg_typename in template_typenames:
        idx = template_typenames.index(str_arg_typename)
        return parser.Type(
            typename=instantiations[idx],
            is_const=ctype.is_const,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basis=ctype.is_basis,
        )
    elif str_arg_typename == 'This':
        # import sys
        if instantiated_class:
            name = instantiated_class.original.name
            namespaces_name = instantiated_class.namespaces()
            namespaces_name.append(name)
            # print("INST: {}, {}, CPP: {}, CLS: {}".format(
            #     ctype, instantiations, cpp_typename, instantiated_class.instantiations
            # ), file=sys.stderr)
            cpp_typename = parser.Typename(
                namespaces_name, instantiations=[inst for inst in instantiated_class.instantiations]
            )
        return parser.Type(
            typename=cpp_typename,
            is_const=ctype.is_const,
            is_ptr=ctype.is_ptr,
            is_ref=ctype.is_ref,
            is_basis=ctype.is_basis,
        )
    else:
        return ctype


def instantiate_args_list(args_list, template_typenames, instantiations,
                          cpp_typename):
    """
    Instantiate template typenames in an argument list.
    Type with name `This` will be replaced by @p `cpp_typename`.

    @param[in] args_list A list of `parser.Argument` to instantiate.
    @param[in] template_typenames List of template typenames to instantiate,
        e.g. ['T', 'U', 'V'].
    @param[in] instantiations List of specific types to instantiate, each
        associated with each template typename. Each type is a parser.Typename,
        including its name and full namespaces.
    @param[in] cpp_typename Full-namespace cpp class name of this instantiation
        to replace for arguments of type named `This`.
    @return A new list of parser.Argument which types are replaced with their
        instantiations.
    """
    instantiated_args = []
    for arg in args_list:
        new_type = instantiate_type(
            arg.ctype, template_typenames, instantiations, cpp_typename)
        instantiated_args.append(
            parser.Argument(name=arg.name, ctype=new_type))
    return instantiated_args


def instantiate_return_type(return_type, template_typenames, instantiations,
                            cpp_typename, instantiated_class=None):
    new_type1 = instantiate_type(
        return_type.type1, template_typenames, instantiations, cpp_typename, instantiated_class=instantiated_class)
    if return_type.type2:
        new_type2 = instantiate_type(
            return_type.type2, template_typenames, instantiations,
            cpp_typename, instantiated_class=instantiated_class)
    else:
        new_type2 = ''
    return parser.ReturnType(new_type1, new_type2)


def instantiate_name(original_name, instantiations):
    """
    Concatenate instantiated types with an @p original name to form a new
    instantiated name.
    TODO(duy): To avoid conflicts, we should include the instantiation's
    namespaces, but I find that too verbose.
    """
    inst_name = ''

    return "{}{}".format(original_name, "".join(
        [inst.instantiated_name() for inst in instantiations]))


class InstantiatedMethod(parser.Method):
    """
    We can only instantiate template methods with a single template parameter.
    """

    def __init__(self, original, instantiation=''):
        self.original = original
        self.instantiation = instantiation
        self.template = ''
        self.is_const = original.is_const
        self.parent = original.parent

        if not original.template:
            self.name = original.name
            self.return_type = original.return_type
            self.args = original.args
        else:
            if len(self.original.template.typenames) > 1:
                raise ValueError("Can only instantiate template method with "
                                 "single template parameter.")
            self.name = instantiate_name(original.name, [self.instantiation])
            self.return_type = instantiate_return_type(
                original.return_type,
                [self.original.template.typenames[0]],
                [self.instantiation],
                # Keyword type name `This` should already be replaced in the
                # previous class template instantiation round.
                cpp_typename='',
            )
            instantiated_args = instantiate_args_list(
                original.args.args_list,
                [self.original.template.typenames[0]],
                [self.instantiation],
                # Keyword type name `This` should already be replaced in the
                # previous class template instantiation round.
                cpp_typename='',
            )
            self.args = parser.ArgumentList(instantiated_args)

    def to_cpp(self):
        if self.original.template:
            return "{}<{}>".format(self.original.name, self.instantiation)
        else:
            return self.original.name

    def __repr__(self):
        return "Instantiated {}".format(
            super(InstantiatedMethod, self).__repr__()
        )


class InstantiatedClass(parser.Class):
    def __init__(self, original, instantiations=[], new_name=''):
        """
        Template <T, U>
        Instantiations: [T1, U1]
        """
        self.original = original
        self.instantiations = instantiations

        self.template = ''
        self.is_virtual = original.is_virtual
        self.parent_class = original.parent_class
        self.parent = original.parent

        if not original.template:
            self.name = original.name
            self.ctors = list(original.ctors)
            self.static_methods = list(original.static_methods)
            class_instantiated_methods = list(original.methods)
            self.properties = list(original.properties)
        else:
            # Check conditions.
            assert len(original.template.typenames) == len(
                instantiations), "Typenames and instantiations mismatch!"

            self.name = instantiate_name(
                original.name, instantiations) if not new_name else new_name
            self.ctors = self.instantiate_ctors()
            self.static_methods = self.instantiate_static_methods()
            class_instantiated_methods = \
                self.instantiate_class_templates_in_methods()
            self.properties = self.instantiate_properties()

        # Second instantiation round to instantiate template methods.
        self.methods = []
        for method in class_instantiated_methods:
            if not method.template:
                self.methods.append(InstantiatedMethod(method, ''))
            else:
                assert len(
                    method.template.typenames) == 1, ""\
                    "Can only instantiate single template methods"
                for inst in method.template.instantiations[0]:
                    self.methods.append(InstantiatedMethod(method, inst))

    def __repr__(self):
        return "{virtual} class {name} [{cpp_class}]: {parent_class}\n"\
            "{ctors}\n{static_methods}\n{methods}".format(
               virtual="virtual" if self.is_virtual else '',
               name=self.name,
               cpp_class=self.cpp_class(),
               parent_class=self.parent,
               ctors="\n".join([ctor.__repr__() for ctor in self.ctors]),
               methods="\n".join([m.__repr__() for m in self.methods]),
               static_methods="\n".join([m.__repr__()
                                         for m in self.static_methods]),
            )

    def instantiate_ctors(self):
        instantiated_ctors = []
        for ctor in self.original.ctors:
            instantiated_args = instantiate_args_list(
                ctor.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                self.cpp_typename(),
            )
            instantiated_ctors.append(parser.Constructor(
                name=self.name,
                args=parser.ArgumentList(instantiated_args),
                parent=self,
            ))
        return instantiated_ctors

    def instantiate_static_methods(self):
        instantiated_static_methods = []
        for static_method in self.original.static_methods:
            instantiated_args = instantiate_args_list(
                static_method.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                self.cpp_typename()
            )
            instantiated_static_methods.append(
                parser.StaticMethod(
                    name=static_method.name,
                    return_type=instantiate_return_type(
                        static_method.return_type,
                        self.original.template.typenames,
                        self.instantiations,
                        self.cpp_typename(),
                        instantiated_class=self
                    ),
                    args=parser.ArgumentList(instantiated_args),
                    parent=self,
                )
            )
        return instantiated_static_methods

    def instantiate_class_templates_in_methods(self):
        """
        This function only instantiates class templates in the methods.
        Template methods are instantiated in InstantiatedMethod in the second
        round.
        """
        class_instantiated_methods = []
        for method in self.original.methods:
            instantiated_args = instantiate_args_list(
                method.args.args_list,
                self.original.template.typenames,
                self.instantiations,
                self.cpp_typename(),
            )
            class_instantiated_methods.append(parser.Method(
                template=method.template,
                name=method.name,
                return_type=instantiate_return_type(
                    method.return_type,
                    self.original.template.typenames,
                    self.instantiations,
                    self.cpp_typename(),
                ),
                args=parser.ArgumentList(instantiated_args),
                is_const=method.is_const,
                parent=self,
            ))
        return class_instantiated_methods

    def instantiate_properties(self):
        instantiated_properties = instantiate_args_list(
            self.original.properties,
            self.original.template.typenames,
            self.instantiations,
            self.cpp_typename(),
        )
        return instantiated_properties

    def cpp_class(self):
        return self.cpp_typename().to_cpp()

    def cpp_typename(self):
        """
        Return a parser.Typename including namespaces and cpp name of this
        class.
        """
        if self.original.template:
            name = "{}<{}>".format(
                self.original.name,
                ", ".join([inst.to_cpp() for inst in self.instantiations]))
        else:
            name = self.original.name
        namespaces_name = self.namespaces()
        namespaces_name.append(name)
        return parser.Typename(namespaces_name)


def instantiate_namespace_inplace(namespace):
    """
    @param[in/out] namespace The namespace which content will be replaced with
        the instantiated content.
    """
    instantiated_content = []
    typedef_content = []

    for element in namespace.content:
        if isinstance(element, parser.Class):
            original_class = element
            if not original_class.template:
                instantiated_content.append(
                    InstantiatedClass(original_class, []))
            else:
                if (len(original_class.template.typenames) > 1
                        and original_class.template.instantiations[0]):
                    raise ValueError(
                        "Can't instantiate multi-parameter templates here. "
                        "Please use typedef template instantiation."
                    )
                for inst in original_class.template.instantiations[0]:
                    instantiated_content.append(
                        InstantiatedClass(original_class, [inst]))
        elif isinstance(element, parser.TypedefTemplateInstantiation):
            typedef_inst = element
            original_class = namespace.top_level().find_class(
                typedef_inst.typename)
            typedef_content.append(
                InstantiatedClass(
                    original_class,
                    typedef_inst.typename.instantiations,
                    typedef_inst.new_name
                )
            )
        elif isinstance(element, parser.Namespace):
            instantiate_namespace_inplace(element)
            instantiated_content.append(element)
        else:
            instantiated_content.append(element)

    instantiated_content.extend(typedef_content)
    namespace.content = instantiated_content
