import textwrap


class WrapperTemplate:
    """Class to encapsulate string templates for use in wrapper generation"""
    boost_headers = textwrap.dedent("""
            #include <boost/archive/text_iarchive.hpp>
            #include <boost/archive/text_oarchive.hpp>
            #include <boost/serialization/export.hpp>
        """)

    typdef_collectors = textwrap.dedent('''\
                typedef std::set<std::shared_ptr<{class_name_sep}>*> Collector_{class_name};
                static Collector_{class_name} collector_{class_name};
            ''')

    delete_obj = textwrap.indent(textwrap.dedent('''\
                {{ for(Collector_{class_name}::iterator iter = collector_{class_name}.begin();
                    iter != collector_{class_name}.end(); ) {{
                  delete *iter;
                  collector_{class_name}.erase(iter++);
                  anyDeleted = true;
                }} }}
            '''),
                                 prefix='  ')

    delete_all_objects = textwrap.dedent('''
            void _deleteAllObjects()
            {{
              mstream mout;
              std::streambuf *outbuf = std::cout.rdbuf(&mout);\n
              bool anyDeleted = false;
            {delete_objs}
              if(anyDeleted)
                cout <<
                  "WARNING:  Wrap modules with variables in the workspace have been reloaded due to\\n"
                  "calling destructors, call \'clear all\' again if you plan to now recompile a wrap\\n"
                  "module, so that your recompiled module is used instead of the old one." << endl;
              std::cout.rdbuf(outbuf);
            }}
        ''')

    rtti_register = textwrap.dedent('''\
            void _{module_name}_RTTIRegister() {{
              const mxArray *alreadyCreated = mexGetVariablePtr("global", "gtsam_{module_name}_rttiRegistry_created");
              if(!alreadyCreated) {{
                std::map<std::string, std::string> types;

            {rtti_classes}

                mxArray *registry = mexGetVariable("global", "gtsamwrap_rttiRegistry");
                if(!registry)
                  registry = mxCreateStructMatrix(1, 1, 0, NULL);
                typedef std::pair<std::string, std::string> StringPair;
                for(const StringPair& rtti_matlab: types) {{
                  int fieldId = mxAddField(registry, rtti_matlab.first.c_str());
                  if(fieldId < 0) {{
                    mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
                  }}
                  mxArray *matlabName = mxCreateString(rtti_matlab.second.c_str());
                  mxSetFieldByNumber(registry, 0, fieldId, matlabName);
                }}
                if(mexPutVariable("global", "gtsamwrap_rttiRegistry", registry) != 0) {{
                  mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
                }}
                mxDestroyArray(registry);

                mxArray *newAlreadyCreated = mxCreateNumericMatrix(0, 0, mxINT8_CLASS, mxREAL);
                if(mexPutVariable("global", "gtsam_{module_name}_rttiRegistry_created", newAlreadyCreated) != 0) {{
                  mexErrMsgTxt("gtsam wrap:  Error indexing RTTI types, inheritance will not work correctly");
                }}
                mxDestroyArray(newAlreadyCreated);
              }}
            }}
        ''')

    collector_function_upcast_from_void = textwrap.dedent('''\
            void {class_name}_upcastFromVoid_{id}(int nargout, mxArray *out[], int nargin, const mxArray *in[]) {{
              mexAtExit(&_deleteAllObjects);
              typedef std::shared_ptr<{cpp_name}> Shared;
              std::shared_ptr<void> *asVoid = *reinterpret_cast<std::shared_ptr<void>**> (mxGetData(in[0]));
              out[0] = mxCreateNumericMatrix(1, 1, mxUINT32OR64_CLASS, mxREAL);
              Shared *self = new Shared(std::static_pointer_cast<{cpp_name}>(*asVoid));
              *reinterpret_cast<Shared**>(mxGetData(out[0])) = self;
            }}\n
        ''')

    class_serialize_method = textwrap.dedent('''\
            function varargout = string_serialize(this, varargin)
              % STRING_SERIALIZE usage: string_serialize() : returns string
              % Doxygen can be found at https://gtsam.org/doxygen/
              if length(varargin) == 0
                varargout{{1}} = {wrapper}({wrapper_id}, this, varargin{{:}});
              else
                error('Arguments do not match any overload of function {class_name}.string_serialize');
              end
            end\n
            function sobj = saveobj(obj)
              % SAVEOBJ Saves the object to a matlab-readable format
              sobj = obj.string_serialize();
            end
        ''')

    collector_function_serialize = textwrap.indent(textwrap.dedent("""\
            typedef std::shared_ptr<{full_name}> Shared;
            checkArguments("string_serialize",nargout,nargin-1,0);
            Shared obj = unwrap_shared_ptr<{full_name}>(in[0], "ptr_{namespace}{class_name}");
            ostringstream out_archive_stream;
            boost::archive::text_oarchive out_archive(out_archive_stream);
            out_archive << *obj;
            out[0] = wrap< string >(out_archive_stream.str());
        """),
                                                   prefix='  ')

    collector_function_deserialize = textwrap.indent(textwrap.dedent("""\
            typedef std::shared_ptr<{full_name}> Shared;
            checkArguments("{namespace}{class_name}.string_deserialize",nargout,nargin,1);
            string serialized = unwrap< string >(in[0]);
            istringstream in_archive_stream(serialized);
            boost::archive::text_iarchive in_archive(in_archive_stream);
            Shared output(new {full_name}());
            in_archive >> *output;
            out[0] = wrap_shared_ptr(output,"{namespace}.{class_name}", false);
        """),
                                                     prefix='  ')

    mex_function = textwrap.dedent('''
            void mexFunction(int nargout, mxArray *out[], int nargin, const mxArray *in[])
            {{
              mstream mout;
              std::streambuf *outbuf = std::cout.rdbuf(&mout);\n
              _{module_name}_RTTIRegister();\n
              int id = unwrap<int>(in[0]);\n
              try {{
                switch(id) {{
            {cases}    }}
              }} catch(const std::exception& e) {{
                mexErrMsgTxt(("Exception from gtsam:\\n" + std::string(e.what()) + "\\n").c_str());
              }}\n
              std::cout.rdbuf(outbuf);
            }}
        ''')

    collector_function_shared_return = textwrap.indent(textwrap.dedent('''\
            {{
            std::shared_ptr<{name}> shared({shared_obj});
            out[{id}] = wrap_shared_ptr(shared,"{name}");
            }}{new_line}'''),
                                                       prefix='  ')

    matlab_deserialize = textwrap.indent(textwrap.dedent("""\
                function varargout = string_deserialize(varargin)
                  % STRING_DESERIALIZE usage: string_deserialize() : returns {class_name}
                  % Doxygen can be found at https://gtsam.org/doxygen/
                  if length(varargin) == 1
                    varargout{{1}} = {wrapper}({id}, varargin{{:}});
                  else
                    error('Arguments do not match any overload of function {class_name}.string_deserialize');
                  end
                end\n
                function obj = loadobj(sobj)
                  % LOADOBJ Saves the object to a matlab-readable format
                  obj = {class_name}.string_deserialize(sobj);
                end
            """),
                                         prefix='  ')
