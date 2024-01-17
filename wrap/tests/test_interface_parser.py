"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Tests for interface_parser.

Author: Varun Agrawal
"""

# pylint: disable=import-error,wrong-import-position

import os
import sys
import unittest

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gtwrap.interface_parser import (ArgumentList, Class, Constructor,
                                     DunderMethod, Enum, Enumerator,
                                     ForwardDeclaration, GlobalFunction,
                                     Include, Method, Module, Namespace,
                                     Operator, ReturnType, StaticMethod,
                                     TemplatedType, Type,
                                     TypedefTemplateInstantiation, Typename,
                                     Variable)
from gtwrap.template_instantiator.classes import InstantiatedClass


class TestInterfaceParser(unittest.TestCase):
    """Test driver for all classes in interface_parser.py."""

    def test_typename(self):
        """Test parsing of Typename."""
        typename = Typename.rule.parseString("size_t")[0]
        self.assertEqual("size_t", typename.name)

    def test_basic_type(self):
        """Tests for BasicType."""
        # Check basic type
        t = Type.rule.parseString("int x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_basic)

        # Check const
        t = Type.rule.parseString("const int x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_basic)
        self.assertTrue(t.is_const)

        # Check shared pointer
        t = Type.rule.parseString("int* x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_shared_ptr)

        # Check raw pointer
        t = Type.rule.parseString("int@ x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_ptr)

        # Check reference
        t = Type.rule.parseString("int& x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_ref)

        # Check const reference
        t = Type.rule.parseString("const int& x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_const)
        self.assertTrue(t.is_ref)

    def test_custom_type(self):
        """Tests for CustomType."""
        # Check qualified type
        t = Type.rule.parseString("gtsam::Pose3 x")[0]
        self.assertEqual("Pose3", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertTrue(not t.is_basic)

        # Check const
        t = Type.rule.parseString("const gtsam::Pose3 x")[0]
        self.assertEqual("Pose3", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertTrue(t.is_const)

        # Check shared pointer
        t = Type.rule.parseString("gtsam::Pose3* x")[0]
        self.assertEqual("Pose3", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertTrue(t.is_shared_ptr)
        self.assertEqual("std::shared_ptr<gtsam::Pose3>", t.to_cpp())

        # Check raw pointer
        t = Type.rule.parseString("gtsam::Pose3@ x")[0]
        self.assertEqual("Pose3", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertTrue(t.is_ptr)

        # Check reference
        t = Type.rule.parseString("gtsam::Pose3& x")[0]
        self.assertEqual("Pose3", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertTrue(t.is_ref)

        # Check const reference
        t = Type.rule.parseString("const gtsam::Pose3& x")[0]
        self.assertEqual("Pose3", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertTrue(t.is_const)
        self.assertTrue(t.is_ref)

    def test_templated_type(self):
        """Test a templated type."""
        t = TemplatedType.rule.parseString("Eigen::Matrix<double, 3, 4>")[0]
        self.assertEqual("Matrix", t.typename.name)
        self.assertEqual(["Eigen"], t.typename.namespaces)
        self.assertEqual("double", t.typename.instantiations[0].name)
        self.assertEqual("3", t.typename.instantiations[1].name)
        self.assertEqual("4", t.typename.instantiations[2].name)

        t = TemplatedType.rule.parseString(
            "gtsam::PinholeCamera<gtsam::Cal3S2>")[0]
        self.assertEqual("PinholeCamera", t.typename.name)
        self.assertEqual(["gtsam"], t.typename.namespaces)
        self.assertEqual("Cal3S2", t.typename.instantiations[0].name)
        self.assertEqual(["gtsam"], t.typename.instantiations[0].namespaces)

        t = TemplatedType.rule.parseString("PinholeCamera<Cal3S2*>")[0]
        self.assertEqual("PinholeCamera", t.typename.name)
        self.assertEqual("Cal3S2", t.typename.instantiations[0].name)
        self.assertTrue(t.template_params[0].is_shared_ptr)

    def test_empty_arguments(self):
        """Test no arguments."""
        empty_args = ArgumentList.rule.parseString("")[0]
        self.assertEqual(0, len(empty_args))

    def test_argument_list(self):
        """Test arguments list for a method/function."""
        arg_string = "int a, C1 c1, C2& c2, C3* c3, "\
            "const C4 c4, const C5& c5,"\
            "const C6* c6"
        args = ArgumentList.rule.parseString(arg_string)[0]

        self.assertEqual(7, len(args.list()))
        self.assertEqual(['a', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6'],
                         args.names())

    def test_argument_list_qualifiers(self):
        """
        Test arguments list where the arguments are qualified with `const`
        and can be either raw pointers, shared pointers or references.
        """
        arg_string = "double x1, double* x2, double& x3, double@ x4, " \
            "const double x5, const double* x6, const double& x7, const double@ x8"
        args = ArgumentList.rule.parseString(arg_string)[0].list()
        self.assertEqual(8, len(args))
        self.assertFalse(args[1].ctype.is_ptr and args[1].ctype.is_shared_ptr
                         and args[1].ctype.is_ref)
        self.assertTrue(args[1].ctype.is_shared_ptr)
        self.assertTrue(args[2].ctype.is_ref)
        self.assertTrue(args[3].ctype.is_ptr)
        self.assertTrue(args[4].ctype.is_const)
        self.assertTrue(args[5].ctype.is_shared_ptr and args[5].ctype.is_const)
        self.assertTrue(args[6].ctype.is_ref and args[6].ctype.is_const)
        self.assertTrue(args[7].ctype.is_ptr and args[7].ctype.is_const)

    def test_argument_list_templated(self):
        """Test arguments list where the arguments can be templated."""
        arg_string = "std::pair<string, double> steps, vector<T*> vector_of_pointers"
        args = ArgumentList.rule.parseString(arg_string)[0]
        args_list = args.list()
        self.assertEqual(2, len(args_list))
        self.assertEqual("std::pair<string, double>",
                         args_list[0].ctype.to_cpp())
        self.assertEqual("vector<std::shared_ptr<T>>",
                         args_list[1].ctype.to_cpp())

    def test_default_arguments(self):
        """Tests any expression that is a valid default argument"""
        args = ArgumentList.rule.parseString("""
            string c = "", int z = 0, double z2 = 0.0, bool f = false,
            string s="hello"+"goodbye", char c='a', int a=3,
            int b, double pi = 3.1415""")[0].list()

        # Test for basic types
        self.assertEqual(args[0].default, '""')
        self.assertEqual(args[1].default, '0')
        self.assertEqual(args[2].default, '0.0')
        self.assertEqual(args[3].default, "false")
        self.assertEqual(args[4].default, '"hello"+"goodbye"')
        self.assertEqual(args[5].default, "'a'")
        self.assertEqual(args[6].default, '3')
        # No default argument should set `default` to None
        self.assertIsNone(args[7].default)
        self.assertEqual(args[8].default, '3.1415')

        arg0 = 'gtsam::DefaultKeyFormatter'
        arg1 = 'std::vector<size_t>()'
        arg2 = '{1, 2}'
        arg3 = '[&c1, &c2](string s=5, int a){return s+"hello"+a+c1+c2;}'
        arg4 = 'gtsam::Pose3()'
        arg5 = 'Factor<gtsam::Pose3, gtsam::Point3>()'
        arg6 = 'gtsam::Point3(1, 2, 3)'
        arg7 = 'ns::Class<T, U>(3, 2, 1, "name")'

        argument_list = """
            gtsam::KeyFormatter kf = {arg0},
            std::vector<size_t> v = {arg1},
            std::vector<size_t> l = {arg2},
            gtsam::KeyFormatter lambda = {arg3},
            gtsam::Pose3 p = {arg4},
            Factor<gtsam::Pose3, gtsam::Point3> x = {arg5},
            gtsam::Point3 x = {arg6},
            ns::Class<T, U> obj = {arg7}
            """.format(arg0=arg0,
                       arg1=arg1,
                       arg2=arg2,
                       arg3=arg3,
                       arg4=arg4,
                       arg5=arg5,
                       arg6=arg6,
                       arg7=arg7)
        args = ArgumentList.rule.parseString(argument_list)[0].list()

        # Test non-basic type
        self.assertEqual(args[0].default, arg0)
        # Test templated type
        self.assertEqual(args[1].default, arg1)
        self.assertEqual(args[2].default, arg2)
        self.assertEqual(args[3].default, arg3)
        self.assertEqual(args[4].default, arg4)
        self.assertEqual(args[5].default, arg5)
        self.assertEqual(args[6].default, arg6)
        # Test for default argument with multiple templates and params
        self.assertEqual(args[7].default, arg7)

    def test_return_type(self):
        """Test ReturnType"""
        # Test void
        return_type = ReturnType.rule.parseString("void")[0]
        self.assertEqual("void", return_type.type1.typename.name)
        self.assertTrue(return_type.type1.is_basic)

        # Test basic type
        return_type = ReturnType.rule.parseString("size_t")[0]
        self.assertEqual("size_t", return_type.type1.typename.name)
        self.assertTrue(not return_type.type2)
        self.assertTrue(return_type.type1.is_basic)

        # Test with qualifiers
        return_type = ReturnType.rule.parseString("int&")[0]
        self.assertEqual("int", return_type.type1.typename.name)
        self.assertTrue(return_type.type1.is_basic
                        and return_type.type1.is_ref)

        return_type = ReturnType.rule.parseString("const int")[0]
        self.assertEqual("int", return_type.type1.typename.name)
        self.assertTrue(return_type.type1.is_basic
                        and return_type.type1.is_const)

        # Test pair return
        return_type = ReturnType.rule.parseString("pair<char, int>")[0]
        self.assertEqual("char", return_type.type1.typename.name)
        self.assertEqual("int", return_type.type2.typename.name)

        return_type = ReturnType.rule.parseString("pair<Test ,Test*>")[0]
        self.assertEqual("Test", return_type.type1.typename.name)
        self.assertEqual("Test", return_type.type2.typename.name)
        self.assertTrue(return_type.type2.is_shared_ptr)

    def test_method(self):
        """Test for a class method."""
        ret = Method.rule.parseString("int f();")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(0, len(ret.args))
        self.assertTrue(not ret.is_const)

        ret = Method.rule.parseString("int f() const;")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(0, len(ret.args))
        self.assertTrue(ret.is_const)

        ret = Method.rule.parseString(
            "int f(const int x, const Class& c, Class* t) const;")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(3, len(ret.args))

        ret = Method.rule.parseString(
            "pair<First ,Second*> create_MixedPtrs();")[0]
        self.assertEqual("create_MixedPtrs", ret.name)
        self.assertEqual(0, len(ret.args))
        self.assertEqual("First", ret.return_type.type1.typename.name)
        self.assertEqual("Second", ret.return_type.type2.typename.name)

    def test_static_method(self):
        """Test for static methods."""
        ret = StaticMethod.rule.parseString("static int f();")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(0, len(ret.args))

        ret = StaticMethod.rule.parseString(
            "static int f(const int x, const Class& c, Class* t);")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(3, len(ret.args))

    def test_constructor(self):
        """Test for class constructor."""
        ret = Constructor.rule.parseString("f();")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(0, len(ret.args))

        ret = Constructor.rule.parseString(
            "f(const int x, const Class& c, Class* t);")[0]
        self.assertEqual("f", ret.name)
        self.assertEqual(3, len(ret.args))

        ret = Constructor.rule.parseString(
            """ForwardKinematics(const gtdynamics::Robot& robot,
                    const string& start_link_name, const string& end_link_name,
                    const gtsam::Values& joint_angles,
                    const gtsam::Pose3& l2Tp = gtsam::Pose3());""")[0]
        self.assertEqual("ForwardKinematics", ret.name)
        self.assertEqual(5, len(ret.args))
        self.assertEqual("gtsam::Pose3()", ret.args.list()[4].default)

    def test_constructor_templated(self):
        """Test for templated class constructor."""
        f = """
        template<T = {double, int}>
        Class();
        """
        ret = Constructor.rule.parseString(f)[0]
        self.assertEqual("Class", ret.name)
        self.assertEqual(0, len(ret.args))

        f = """
        template<T = {double, int}>
        Class(const T& name);
        """
        ret = Constructor.rule.parseString(f)[0]
        self.assertEqual("Class", ret.name)
        self.assertEqual(1, len(ret.args))
        self.assertEqual("const T & name", ret.args.args_list[0].to_cpp())

    def test_dunder_method(self):
        """Test for special python dunder methods."""
        iter_string = "__iter__();"
        ret = DunderMethod.rule.parse_string(iter_string)[0]
        self.assertEqual("iter", ret.name)

        contains_string = "__contains__(size_t key);"
        ret = DunderMethod.rule.parse_string(contains_string)[0]
        self.assertEqual("contains", ret.name)
        self.assertTrue(len(ret.args) == 1)

    def test_operator_overload(self):
        """Test for operator overloading."""
        # Unary operator
        wrap_string = "gtsam::Vector2 operator-() const;"
        ret = Operator.rule.parseString(wrap_string)[0]
        self.assertEqual("operator", ret.name)
        self.assertEqual("-", ret.operator)
        self.assertEqual("Vector2", ret.return_type.type1.typename.name)
        self.assertEqual("gtsam::Vector2",
                         ret.return_type.type1.typename.to_cpp())
        self.assertTrue(len(ret.args) == 0)
        self.assertTrue(ret.is_unary)

        # Binary operator
        wrap_string = "gtsam::Vector2 operator*(const gtsam::Vector2 &v) const;"
        ret = Operator.rule.parseString(wrap_string)[0]
        self.assertEqual("operator", ret.name)
        self.assertEqual("*", ret.operator)
        self.assertEqual("Vector2", ret.return_type.type1.typename.name)
        self.assertEqual("gtsam::Vector2",
                         ret.return_type.type1.typename.to_cpp())
        self.assertTrue(len(ret.args) == 1)
        self.assertEqual("const gtsam::Vector2 &",
                         repr(ret.args.list()[0].ctype))
        self.assertTrue(not ret.is_unary)

    def test_typedef_template_instantiation(self):
        """Test for typedef'd instantiation of a template."""
        typedef = TypedefTemplateInstantiation.rule.parseString("""
        typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>
            BearingFactor2D;
        """)[0]
        self.assertEqual("BearingFactor2D", typedef.new_name)
        self.assertEqual("BearingFactor", typedef.typename.name)
        self.assertEqual(["gtsam"], typedef.typename.namespaces)
        self.assertEqual(3, len(typedef.typename.instantiations))

    def test_base_class(self):
        """Test a base class."""
        ret = Class.rule.parseString("""
            virtual class Base {
            };
            """)[0]
        self.assertEqual("Base", ret.name)
        self.assertEqual(0, len(ret.ctors))
        self.assertEqual(0, len(ret.methods))
        self.assertEqual(0, len(ret.static_methods))
        self.assertEqual(0, len(ret.properties))
        self.assertTrue(ret.is_virtual)

    def test_empty_class(self):
        """Test an empty class declaration."""
        ret = Class.rule.parseString("""
            class FactorIndices {};
        """)[0]
        self.assertEqual("FactorIndices", ret.name)
        self.assertEqual(0, len(ret.ctors))
        self.assertEqual(0, len(ret.methods))
        self.assertEqual(0, len(ret.static_methods))
        self.assertEqual(0, len(ret.properties))
        self.assertTrue(not ret.is_virtual)

    def test_class(self):
        """Test a non-trivial class."""
        ret = Class.rule.parseString("""
        class SymbolicFactorGraph {
            SymbolicFactorGraph();
            SymbolicFactorGraph(const gtsam::SymbolicBayesNet& bayesNet);
            SymbolicFactorGraph(const gtsam::SymbolicBayesTree& bayesTree);

            // Dummy static method
            static gtsam::SymbolidFactorGraph CreateGraph();

            void push_back(gtsam::SymbolicFactor* factor);
            void print(string s) const;
            bool equals(const gtsam::SymbolicFactorGraph& rhs, double tol) const;
            size_t size() const;
            bool exists(size_t idx) const;

            // Standard interface
            gtsam::KeySet keys() const;
            void push_back(const gtsam::SymbolicFactorGraph& graph);
            void push_back(const gtsam::SymbolicBayesNet& bayesNet);
            void push_back(const gtsam::SymbolicBayesTree& bayesTree);

            /* Advanced interface */
            void push_factor(size_t key);
            void push_factor(size_t key1, size_t key2);
            void push_factor(size_t key1, size_t key2, size_t key3);
            void push_factor(size_t key1, size_t key2, size_t key3, size_t key4);

            gtsam::SymbolicBayesNet* eliminateSequential();
            gtsam::SymbolicBayesNet* eliminateSequential(
                const gtsam::Ordering& ordering);
            gtsam::SymbolicBayesTree* eliminateMultifrontal();
            gtsam::SymbolicBayesTree* eliminateMultifrontal(
                const gtsam::Ordering& ordering);
            pair<gtsam::SymbolicBayesNet*, gtsam::SymbolicFactorGraph*>
                eliminatePartialSequential(const gtsam::Ordering& ordering);
            pair<gtsam::SymbolicBayesNet*, gtsam::SymbolicFactorGraph*>
                eliminatePartialSequential(const gtsam::KeyVector& keys);
            pair<gtsam::SymbolicBayesTree*, gtsam::SymbolicFactorGraph*>
                eliminatePartialMultifrontal(const gtsam::Ordering& ordering);
            gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(
                const gtsam::Ordering& ordering);
            gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(
                const gtsam::KeyVector& key_vector,
                const gtsam::Ordering& marginalizedVariableOrdering);
            gtsam::SymbolicFactorGraph* marginal(const gtsam::KeyVector& key_vector);
            };
        """)[0]

        self.assertEqual("SymbolicFactorGraph", ret.name)
        self.assertEqual(3, len(ret.ctors))
        self.assertEqual(23, len(ret.methods))
        self.assertEqual(1, len(ret.static_methods))
        self.assertEqual(0, len(ret.properties))
        self.assertTrue(not ret.is_virtual)

    def test_templated_class(self):
        """Test a templated class."""
        ret = Class.rule.parseString("""
        template<POSE, POINT>
        class MyFactor {};
        """)[0]

        self.assertEqual("MyFactor", ret.name)
        self.assertEqual("<POSE, POINT>", repr(ret.template))

    def test_class_inheritance(self):
        """Test for class inheritance."""
        ret = Class.rule.parseString("""
        virtual class Null: gtsam::noiseModel::mEstimator::Base {
          Null();
          void print(string s) const;
          static gtsam::noiseModel::mEstimator::Null* Create();

          // enabling serialization functionality
          void serializable() const;
        };
        """)[0]
        self.assertEqual("Null", ret.name)
        self.assertEqual(1, len(ret.ctors))
        self.assertEqual(2, len(ret.methods))
        self.assertEqual(1, len(ret.static_methods))
        self.assertEqual(0, len(ret.properties))
        self.assertEqual("Base", ret.parent_class.name)
        self.assertEqual(["gtsam", "noiseModel", "mEstimator"],
                         ret.parent_class.namespaces)
        self.assertTrue(ret.is_virtual)

        ret = Class.rule.parseString(
            "class ForwardKinematicsFactor : gtsam::BetweenFactor<gtsam::Pose3> {};"
        )[0]
        ret = InstantiatedClass(ret,
                                [])  # Needed to correctly parse parent class
        self.assertEqual("ForwardKinematicsFactor", ret.name)
        self.assertEqual("BetweenFactor", ret.parent_class.name)
        self.assertEqual(["gtsam"], ret.parent_class.namespaces)
        self.assertEqual("Pose3", ret.parent_class.instantiations[0].name)
        self.assertEqual(["gtsam"],
                         ret.parent_class.instantiations[0].namespaces)

    def test_class_with_enum(self):
        """Test for class with nested enum."""
        ret = Class.rule.parseString("""
        class Pet {
            Pet(const string &name, Kind type);
            enum Kind { Dog, Cat };
        };
        """)[0]
        self.assertEqual(ret.name, "Pet")
        self.assertEqual(ret.enums[0].name, "Kind")

    def test_include(self):
        """Test for include statements."""
        include = Include.rule.parseString(
            "#include <gtsam/slam/PriorFactor.h>")[0]
        self.assertEqual("gtsam/slam/PriorFactor.h", include.header)

    def test_forward_declaration(self):
        """Test for forward declarations."""
        fwd = ForwardDeclaration.rule.parseString(
            "virtual class Test:gtsam::Point3;")[0]

        self.assertEqual("Test", fwd.name)
        self.assertTrue(fwd.is_virtual)

    def test_function(self):
        """Test for global/free function."""
        func = GlobalFunction.rule.parseString("""
        gtsam::Values localToWorld(const gtsam::Values& local,
            const gtsam::Pose2& base, const gtsam::KeyVector& keys);
        """)[0]
        self.assertEqual("localToWorld", func.name)
        self.assertEqual("Values", func.return_type.type1.typename.name)
        self.assertEqual(3, len(func.args))

    def test_global_variable(self):
        """Test for global variable."""
        variable = Variable.rule.parseString("string kGravity;")[0]
        self.assertEqual(variable.name, "kGravity")
        self.assertEqual(variable.ctype.typename.name, "string")

        variable = Variable.rule.parseString("string kGravity = 9.81;")[0]
        self.assertEqual(variable.name, "kGravity")
        self.assertEqual(variable.ctype.typename.name, "string")
        self.assertEqual(variable.default, "9.81")

        variable = Variable.rule.parseString(
            "const string kGravity = 9.81;")[0]
        self.assertEqual(variable.name, "kGravity")
        self.assertEqual(variable.ctype.typename.name, "string")
        self.assertTrue(variable.ctype.is_const)
        self.assertEqual(variable.default, "9.81")

        variable = Variable.rule.parseString(
            "gtsam::Pose3 wTc = gtsam::Pose3();")[0]
        self.assertEqual(variable.name, "wTc")
        self.assertEqual(variable.ctype.typename.name, "Pose3")
        self.assertEqual(variable.default, "gtsam::Pose3()")

        variable = Variable.rule.parseString(
            "gtsam::Pose3 wTc = gtsam::Pose3(1, 2, 0);")[0]
        self.assertEqual(variable.name, "wTc")
        self.assertEqual(variable.ctype.typename.name, "Pose3")
        self.assertEqual(variable.default, "gtsam::Pose3(1, 2, 0)")

    def test_enumerator(self):
        """Test for enumerator."""
        enumerator = Enumerator.rule.parseString("Dog")[0]
        self.assertEqual(enumerator.name, "Dog")

        enumerator = Enumerator.rule.parseString("Cat")[0]
        self.assertEqual(enumerator.name, "Cat")

    def test_enum(self):
        """Test for enums."""
        enum = Enum.rule.parseString("""
        enum Kind {
            Dog,
            Cat
        };
        """)[0]
        self.assertEqual(enum.name, "Kind")
        self.assertEqual(enum.enumerators[0].name, "Dog")
        self.assertEqual(enum.enumerators[1].name, "Cat")

    def test_namespace(self):
        """Test for namespace parsing."""
        namespace = Namespace.rule.parseString("""
        namespace gtsam {
          #include <gtsam/geometry/Point2.h>
          class Point2 {
            Point2();
            Point2(double x, double y);
            double x() const;
            double y() const;
            int dim() const;
            char returnChar() const;
            void argChar(char a) const;
            void argUChar(unsigned char a) const;
          };

          #include <gtsam/geometry/Point3.h>
          class Point3 {
            Point3(double x, double y, double z);
            double norm() const;

            // static functions - use static keyword and uppercase
            static double staticFunction();
            static gtsam::Point3 StaticFunctionRet(double z);

            // enabling serialization functionality
            void serialize() const; // Just triggers a flag internally
          };
        }""")[0]
        self.assertEqual("gtsam", namespace.name)

    def test_module(self):
        """Test module parsing."""
        module = Module.parseString("""
        namespace one {
            namespace two {
                namespace three {
                    class Class123 {
                    };
                }
                class Class12a {
                };
            }
            namespace two_dummy {
                namespace three_dummy{

                }
                namespace fourth_dummy{

                }
            }
            namespace two {
                class Class12b {

                };
            }
            int oneVar;
        }

        class Global{
        };
        int globalVar;
        """)

        self.assertEqual(["one", "Global", "globalVar"],
                         [x.name for x in module.content])
        self.assertEqual(["two", "two_dummy", "two", "oneVar"],
                         [x.name for x in module.content[0].content])


if __name__ == '__main__':
    unittest.main()
