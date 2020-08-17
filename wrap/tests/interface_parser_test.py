# TODO(duy): make them proper tests!!!
import unittest

import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from interface_parser import *


class TestPyparsing(unittest.TestCase):
    def test_argument_list(self):
        arg_string = "int a, C1 c1, C2& c2, C3* c3, "\
            "const C4 c4, const C5& c5,"\
            "const C6* c6"
        args = ArgumentList.rule.parseString(arg_string)
        print(ArgumentList(args))


empty_args = ArgumentList.rule.parseString("")[0]
print(empty_args)

arg_string = "int a, C1 c1, C2& c2, C3* c3, "\
    "const C4 c4, const C5& c5,"\
    "const C6* c6"
args = ArgumentList.rule.parseString(arg_string)[0]
print(args)

# Test ReturnType
ReturnType.rule.parseString("pair<fdsa, rewcds>")[0]
ReturnType.rule.parseString("cdwdc")[0]

# expect throw
# ReturnType.parseString("int&")
# ReturnType.parseString("const int")

ret = Class.rule.parseString("""
virtual class SymbolicFactorGraph {
  SymbolicFactorGraph();
  SymbolicFactorGraph(const gtsam::SymbolicBayesNet& bayesNet);
  SymbolicFactorGraph(const gtsam::SymbolicBayesTree& bayesTree);

  // From FactorGraph.
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

ret = Class.rule.parseString("""
virtual class Base {
};
""")[0]

ret = Class.rule.parseString("""
virtual class Null: gtsam::noiseModel::mEstimator::Base {
  Null();
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Null* Create();

  // enabling serialization functionality
  void serializable() const;
};
""")[0]

retFactorIndices = Class.rule.parseString("""
class FactorIndices {};
""")[0]

retIsam2 = Class.rule.parseString("""
class ISAM2 {
  ISAM2();
  ISAM2(const gtsam::ISAM2Params& params);
  ISAM2(const gtsam::ISAM2& other);

  bool equals(const gtsam::ISAM2& other, double tol) const;
  void print(string s) const;
  void printStats() const;
  void saveGraph(string s) const;

  gtsam::ISAM2Result update();
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
    const gtsam::Values& newTheta);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
    const gtsam::Values& newTheta, const gtsam::FactorIndices&
    removeFactorIndices);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
    const gtsam::Values& newTheta,
    const gtsam::FactorIndices& removeFactorIndices,
    const gtsam::KeyGroupMap& constrainedKeys);

  gtsam::Values getLinearizationPoint() const;
  gtsam::Values calculateEstimate() const;
  template <VALUE = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                     gtsam::Rot3, gtsam::Pose3, gtsam::Cal3_S2, gtsam::Cal3DS2,
                     gtsam::Cal3Bundler, gtsam::EssentialMatrix,
                     gtsam::SimpleCamera, Vector, Matrix}>
  VALUE calculateEstimate(size_t key) const;
  gtsam::Values calculateBestEstimate() const;
  Matrix marginalCovariance(size_t key) const;
  gtsam::VectorValues getDelta() const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
  gtsam::VariableIndex getVariableIndex() const;
  gtsam::ISAM2Params params() const;
};
""")[0]
# if __name__ == '__main__':
#     unittest.main()

typename = Typename.rule.parseString("rew")[0]
ret = ReturnType.rule.parseString("pair<fdsa, rewcds>")[0]
ret1 = Method.rule.parseString(
    "int f(const int x, const Class& c, Class* t) const;")[0]
ret = Method.rule.parseString("int f() const;")[0]

ret1 = StaticMethod.rule.parseString(
    "static int f(const int x, const Class& c, Class* t);")[0]
ret = StaticMethod.rule.parseString("static int f();")[0]
ret1 = Constructor.rule.parseString(
    "f(const int x, const Class& c, Class* t);")[0]
ret = Constructor.rule.parseString("f();")[0]

typedef = TypedefTemplateInstantiation.rule.parseString("""
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>
    BearingFactor2D;
""")[0]

include = Include.rule.parseString("#include <gtsam/slam/PriorFactor.h>")[0]
print(include)

fwd = ForwardDeclaration.rule.parseString(
    "virtual class Test:gtsam::Point3;")[0]

func = GlobalFunction.rule.parseString("""
gtsam::Values localToWorld(const gtsam::Values& local,
    const gtsam::Pose2& base, const gtsam::KeyVector& keys);
""")[0]
print(func)

try:
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
void eigenArguments(Vector v, Matrix m) const;
VectorNotEigen vectorConfusion();
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

}
    """)
except ParseException as pe:
    print(pe.markInputline())

# filename = "tools/workspace/pybind_wrapper/gtsam.h"
# with open(filename, "r") as f:
#     content = f.read()
# module = Module.parseString(content)

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

print("module: ", module)

sub_namespace = find_sub_namespace(module, ['one', 'two', 'three'])
print("Found namespace:", sub_namespace[0].name)
print(find_sub_namespace(module, ['one', 'two_test', 'three']))
print(find_sub_namespace(module, ['one', 'two']))

found_class = module.find_class(
    Typename(namespaces_name=['one', 'two', 'three', 'Class123']))
print(found_class)

found_class = module.find_class(
    Typename(namespaces_name=['one', 'two', 'Class12b']))
print(found_class.name)

found_class = module.find_class(
    Typename(namespaces_name=['one', 'two', 'Class12a']))
print(found_class.name)
