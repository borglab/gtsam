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
                                     ForwardDeclaration, GlobalFunction,
                                     Include, Method, Module, Namespace,
                                     ReturnType, StaticMethod, Type,
                                     TypedefTemplateInstantiation, Typename)


class TestInterfaceParser(unittest.TestCase):
    """Test driver for all classes in interface_parser.py."""
    def test_typename(self):
        """Test parsing of Typename."""
        typename = Typename.rule.parseString("size_t")[0]
        self.assertEqual("size_t", typename.name)

        typename = Typename.rule.parseString(
            "gtsam::PinholeCamera<gtsam::Cal3S2>")[0]
        self.assertEqual("PinholeCamera", typename.name)
        self.assertEqual(["gtsam"], typename.namespaces)
        self.assertEqual("Cal3S2", typename.instantiations[0].name)
        self.assertEqual(["gtsam"], typename.instantiations[0].namespaces)

    def test_type(self):
        """Test for Type."""
        t = Type.rule.parseString("int x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_basis)

        t = Type.rule.parseString("T x")[0]
        self.assertEqual("T", t.typename.name)
        self.assertTrue(not t.is_basis)

        t = Type.rule.parseString("const int x")[0]
        self.assertEqual("int", t.typename.name)
        self.assertTrue(t.is_basis)
        self.assertTrue(t.is_const)

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

        self.assertEqual(7, len(args.args_list))
        self.assertEqual(['a', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6'],
                         args.args_names())

    def test_argument_list_qualifiers(self):
        """
        Test arguments list where the arguments are qualified with `const`
        and can be either raw pointers, shared pointers or references.
        """
        arg_string = "double x1, double* x2, double& x3, double@ x4, " \
            "const double x5, const double* x6, const double& x7, const double@ x8"
        args = ArgumentList.rule.parseString(arg_string)[0].args_list
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

    def test_return_type(self):
        """Test ReturnType"""
        # Test void
        return_type = ReturnType.rule.parseString("void")[0]
        self.assertEqual("void", return_type.type1.typename.name)
        self.assertTrue(return_type.type1.is_basis)

        # Test basis type
        return_type = ReturnType.rule.parseString("size_t")[0]
        self.assertEqual("size_t", return_type.type1.typename.name)
        self.assertTrue(not return_type.type2)
        self.assertTrue(return_type.type1.is_basis)

        # Test with qualifiers
        return_type = ReturnType.rule.parseString("int&")[0]
        self.assertEqual("int", return_type.type1.typename.name)
        self.assertTrue(return_type.type1.is_basis
                        and return_type.type1.is_ref)

        return_type = ReturnType.rule.parseString("const int")[0]
        self.assertEqual("int", return_type.type1.typename.name)
        self.assertTrue(return_type.type1.is_basis
                        and return_type.type1.is_const)

        # Test pair return
        return_type = ReturnType.rule.parseString("pair<char, int>")[0]
        self.assertEqual("char", return_type.type1.typename.name)
        self.assertEqual("int", return_type.type2.typename.name)

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

    def test_include(self):
        """Test for include statements."""
        include = Include.rule.parseString(
            "#include <gtsam/slam/PriorFactor.h>")[0]
        self.assertEqual("gtsam/slam/PriorFactor.h", include.header)

    def test_forward_declaration(self):
        """Test for forward declarations."""
        fwd = ForwardDeclaration.rule.parseString(
            "virtual class Test:gtsam::Point3;")[0]

        fwd_name = fwd.name.asList()[0]
        self.assertEqual("Test", fwd_name.name)
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
        }

        class Global{
        };
        """)

        # print("module: ", module)
        # print(dir(module.content[0].name))
        self.assertEqual(["one", "Global"], [x.name for x in module.content])
        self.assertEqual(["two", "two_dummy", "two"],
                         [x.name for x in module.content[0].content])


if __name__ == '__main__':
    unittest.main()
