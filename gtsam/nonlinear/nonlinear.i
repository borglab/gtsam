//*************************************************************************
// nonlinear
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/basis/ParameterMatrix.h>

#include <gtsam/nonlinear/GraphvizFormatting.h>
class GraphvizFormatting : gtsam::DotWriter {
  GraphvizFormatting();

  enum Axis { X, Y, Z, NEGX, NEGY, NEGZ };
  gtsam::GraphvizFormatting::Axis paperHorizontalAxis;
  gtsam::GraphvizFormatting::Axis paperVerticalAxis;

  double scale;
  bool mergeSimilarFactors;
};

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
class NonlinearFactorGraph {
  NonlinearFactorGraph();
  NonlinearFactorGraph(const gtsam::NonlinearFactorGraph& graph);

  // FactorGraph
  void print(string s = "NonlinearFactorGraph: ",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactorGraph& fg, double tol) const;
  size_t size() const;
  bool empty() const;
  void remove(size_t i);
  void replace(size_t i, gtsam::NonlinearFactor* factors);
  void resize(size_t size);
  size_t nrFactors() const;
  gtsam::NonlinearFactor* at(size_t idx) const;
  void push_back(const gtsam::NonlinearFactorGraph& factors);
  void push_back(gtsam::NonlinearFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  bool exists(size_t idx) const;
  gtsam::KeySet keys() const;
  gtsam::KeyVector keyVector() const;

  template <T = {double,
                 Vector,
                 gtsam::Point2,
                 gtsam::StereoPoint2,
                 gtsam::Point3,
                 gtsam::Rot2,
                 gtsam::SO3,
                 gtsam::SO4,
                 gtsam::Rot3,
                 gtsam::Pose2,
                 gtsam::Pose3,
                 gtsam::Cal3_S2,
                 gtsam::Cal3Fisheye,
                 gtsam::Cal3Unified,
                 gtsam::CalibratedCamera,
                 gtsam::PinholeCamera<gtsam::Cal3_S2>,
                 gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                 gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                 gtsam::PinholeCamera<gtsam::Cal3Unified>,
                 gtsam::imuBias::ConstantBias}>
  void addPrior(size_t key, const T& prior,
                const gtsam::noiseModel::Base* noiseModel);

  // NonlinearFactorGraph
  void printErrors(const gtsam::Values& values) const;
  double error(const gtsam::Values& values) const;
  double probPrime(const gtsam::Values& values) const;
  gtsam::Ordering orderingCOLAMD() const;
  // Ordering* orderingCOLAMDConstrained(const gtsam::Values& c, const
  // std::map<gtsam::Key,int>& constraints) const;
  gtsam::GaussianFactorGraph* linearize(const gtsam::Values& values) const;
  gtsam::NonlinearFactorGraph clone() const;

  string dot(
      const gtsam::Values& values,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::GraphvizFormatting& writer = gtsam::GraphvizFormatting());
  void saveGraph(
      const string& s, const gtsam::Values& values,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::GraphvizFormatting& writer = gtsam::GraphvizFormatting()) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NonlinearFactor {
  // Factor base class
  size_t size() const;
  gtsam::KeyVector keys() const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  void printKeys(string s) const;
  // NonlinearFactor
  bool equals(const gtsam::NonlinearFactor& other, double tol) const;
  double error(const gtsam::Values& c) const;
  size_t dim() const;
  bool active(const gtsam::Values& c) const;
  gtsam::GaussianFactor* linearize(const gtsam::Values& c) const;
  gtsam::NonlinearFactor* clone() const;
  gtsam::NonlinearFactor* rekey(const gtsam::KeyVector& newKeys) const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NoiseModelFactor : gtsam::NonlinearFactor {
  bool equals(const gtsam::NoiseModelFactor& other, double tol) const;
  gtsam::noiseModel::Base* noiseModel() const;
  Vector unwhitenedError(const gtsam::Values& x) const;
  Vector whitenedError(const gtsam::Values& x) const;
};

#include <gtsam/nonlinear/Values.h>
class Values {
  Values();
  Values(const gtsam::Values& other);

  size_t size() const;
  bool empty() const;
  void clear();
  size_t dim() const;

  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::Values& other, double tol) const;

  void insert(const gtsam::Values& values);
  void update(const gtsam::Values& values);
  void insert_or_assign(const gtsam::Values& values);
  void erase(size_t j);
  void swap(gtsam::Values& values);

  bool exists(size_t j) const;
  gtsam::KeyVector keys() const;

  gtsam::VectorValues zeroVectors() const;

  gtsam::Values retract(const gtsam::VectorValues& delta) const;
  gtsam::VectorValues localCoordinates(const gtsam::Values& cp) const;

  // enabling serialization functionality
  void serialize() const;

  // New in 4.0, we have to specialize every insert/update/at to generate
  // wrappers Instead of the old: void insert(size_t j, const gtsam::Value&
  // value); void update(size_t j, const gtsam::Value& val); gtsam::Value
  // at(size_t j) const;

  // The order is important: Vector has to precede Point2/Point3 so `atVector`
  // can work for those fixed-size vectors.
  void insert(size_t j, Vector vector);
  void insert(size_t j, Matrix matrix);
  void insert(size_t j, const gtsam::Point2& point2);
  void insert(size_t j, const gtsam::Point3& point3);
  void insert(size_t j, const gtsam::Rot2& rot2);
  void insert(size_t j, const gtsam::Pose2& pose2);
  void insert(size_t j, const gtsam::SO3& R);
  void insert(size_t j, const gtsam::SO4& Q);
  void insert(size_t j, const gtsam::SOn& P);
  void insert(size_t j, const gtsam::Rot3& rot3);
  void insert(size_t j, const gtsam::Pose3& pose3);
  void insert(size_t j, const gtsam::Unit3& unit3);
  void insert(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void insert(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void insert(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void insert(size_t j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insert(size_t j, const gtsam::Cal3Unified& cal3unified);
  void insert(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insert(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert(size_t j, const gtsam::NavState& nav_state);
  void insert(size_t j, double c);
  void insert(size_t j, const gtsam::ParameterMatrix<1>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<2>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<3>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<4>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<5>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<6>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<7>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<8>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<9>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<10>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<11>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<12>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<13>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<14>& X);
  void insert(size_t j, const gtsam::ParameterMatrix<15>& X);

  template <T = {gtsam::Point2,
                 gtsam::Point3}>
  void insert(size_t j, const T& val);

  void update(size_t j, const gtsam::Point2& point2);
  void update(size_t j, const gtsam::Point3& point3);
  void update(size_t j, const gtsam::Rot2& rot2);
  void update(size_t j, const gtsam::Pose2& pose2);
  void update(size_t j, const gtsam::SO3& R);
  void update(size_t j, const gtsam::SO4& Q);
  void update(size_t j, const gtsam::SOn& P);
  void update(size_t j, const gtsam::Rot3& rot3);
  void update(size_t j, const gtsam::Pose3& pose3);
  void update(size_t j, const gtsam::Unit3& unit3);
  void update(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void update(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void update(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void update(size_t j, const gtsam::Cal3Fisheye& cal3fisheye);
  void update(size_t j, const gtsam::Cal3Unified& cal3unified);
  void update(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void update(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void update(size_t j, const gtsam::NavState& nav_state);
  void update(size_t j, Vector vector);
  void update(size_t j, Matrix matrix);
  void update(size_t j, double c);
  void update(size_t j, const gtsam::ParameterMatrix<1>& X);
  void update(size_t j, const gtsam::ParameterMatrix<2>& X);
  void update(size_t j, const gtsam::ParameterMatrix<3>& X);
  void update(size_t j, const gtsam::ParameterMatrix<4>& X);
  void update(size_t j, const gtsam::ParameterMatrix<5>& X);
  void update(size_t j, const gtsam::ParameterMatrix<6>& X);
  void update(size_t j, const gtsam::ParameterMatrix<7>& X);
  void update(size_t j, const gtsam::ParameterMatrix<8>& X);
  void update(size_t j, const gtsam::ParameterMatrix<9>& X);
  void update(size_t j, const gtsam::ParameterMatrix<10>& X);
  void update(size_t j, const gtsam::ParameterMatrix<11>& X);
  void update(size_t j, const gtsam::ParameterMatrix<12>& X);
  void update(size_t j, const gtsam::ParameterMatrix<13>& X);
  void update(size_t j, const gtsam::ParameterMatrix<14>& X);
  void update(size_t j, const gtsam::ParameterMatrix<15>& X);

  void insert_or_assign(size_t j, const gtsam::Point2& point2);
  void insert_or_assign(size_t j, const gtsam::Point3& point3);
  void insert_or_assign(size_t j, const gtsam::Rot2& rot2);
  void insert_or_assign(size_t j, const gtsam::Pose2& pose2);
  void insert_or_assign(size_t j, const gtsam::SO3& R);
  void insert_or_assign(size_t j, const gtsam::SO4& Q);
  void insert_or_assign(size_t j, const gtsam::SOn& P);
  void insert_or_assign(size_t j, const gtsam::Rot3& rot3);
  void insert_or_assign(size_t j, const gtsam::Pose3& pose3);
  void insert_or_assign(size_t j, const gtsam::Unit3& unit3);
  void insert_or_assign(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void insert_or_assign(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void insert_or_assign(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void insert_or_assign(size_t j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insert_or_assign(size_t j, const gtsam::Cal3Unified& cal3unified);
  void insert_or_assign(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void insert_or_assign(size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insert_or_assign(size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insert_or_assign(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert_or_assign(size_t j, const gtsam::NavState& nav_state);
  void insert_or_assign(size_t j, Vector vector);
  void insert_or_assign(size_t j, Matrix matrix);
  void insert_or_assign(size_t j, double c);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<1>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<2>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<3>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<4>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<5>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<6>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<7>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<8>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<9>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<10>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<11>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<12>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<13>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<14>& X);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix<15>& X);

  template <T = {gtsam::Point2,
                 gtsam::Point3,
                 gtsam::Rot2,
                 gtsam::Pose2,
                 gtsam::SO3,
                 gtsam::SO4,
                 gtsam::SOn,
                 gtsam::Rot3,
                 gtsam::Pose3,
                 gtsam::Unit3,
                 gtsam::Cal3_S2,
                 gtsam::Cal3DS2,
                 gtsam::Cal3Bundler,
                 gtsam::Cal3Fisheye,
                 gtsam::Cal3Unified,
                 gtsam::EssentialMatrix,
                 gtsam::PinholeCamera<gtsam::Cal3_S2>,
                 gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                 gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                 gtsam::PinholeCamera<gtsam::Cal3Unified>,
                 gtsam::PinholePose<gtsam::Cal3_S2>,
                 gtsam::PinholePose<gtsam::Cal3Bundler>,
                 gtsam::PinholePose<gtsam::Cal3Fisheye>,
                 gtsam::PinholePose<gtsam::Cal3Unified>,
                 gtsam::imuBias::ConstantBias,
                 gtsam::NavState,
                 Vector,
                 Matrix,
                 double,
                 gtsam::ParameterMatrix<1>,
                 gtsam::ParameterMatrix<2>,
                 gtsam::ParameterMatrix<3>,
                 gtsam::ParameterMatrix<4>,
                 gtsam::ParameterMatrix<5>,
                 gtsam::ParameterMatrix<6>,
                 gtsam::ParameterMatrix<7>,
                 gtsam::ParameterMatrix<8>,
                 gtsam::ParameterMatrix<9>,
                 gtsam::ParameterMatrix<10>,
                 gtsam::ParameterMatrix<11>,
                 gtsam::ParameterMatrix<12>,
                 gtsam::ParameterMatrix<13>,
                 gtsam::ParameterMatrix<14>,
                 gtsam::ParameterMatrix<15>}>
  T at(size_t j);
};

#include <gtsam/nonlinear/Marginals.h>
class Marginals {
  Marginals(const gtsam::NonlinearFactorGraph& graph,
            const gtsam::Values& solution);
  Marginals(const gtsam::GaussianFactorGraph& gfgraph,
            const gtsam::Values& solution);
  Marginals(const gtsam::GaussianFactorGraph& gfgraph,
            const gtsam::VectorValues& solutionvec);

  void print(string s = "Marginals: ", const gtsam::KeyFormatter& keyFormatter =
                                           gtsam::DefaultKeyFormatter) const;
  Matrix marginalCovariance(size_t variable) const;
  Matrix marginalInformation(size_t variable) const;
  gtsam::JointMarginal jointMarginalCovariance(
      const gtsam::KeyVector& variables) const;
  gtsam::JointMarginal jointMarginalInformation(
      const gtsam::KeyVector& variables) const;
};

class JointMarginal {
  Matrix at(size_t iVariable, size_t jVariable) const;
  Matrix fullMatrix() const;
  void print(string s = "", gtsam::KeyFormatter keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/nonlinear/LinearContainerFactor.h>
virtual class LinearContainerFactor : gtsam::NonlinearFactor {
  LinearContainerFactor(gtsam::GaussianFactor* factor,
                        const gtsam::Values& linearizationPoint);
  LinearContainerFactor(gtsam::GaussianFactor* factor);

  gtsam::GaussianFactor* factor() const;
  //  const std::optional<Values>& linearizationPoint() const;

  bool isJacobian() const;
  gtsam::JacobianFactor* toJacobian() const;
  gtsam::HessianFactor* toHessian() const;

  static gtsam::NonlinearFactorGraph ConvertLinearGraph(
      const gtsam::GaussianFactorGraph& linear_graph,
      const gtsam::Values& linearizationPoint);

  static gtsam::NonlinearFactorGraph ConvertLinearGraph(
      const gtsam::GaussianFactorGraph& linear_graph);

  // enabling serialization functionality
  void serializable() const;
};  // \class LinearContainerFactor

// Summarization functionality
//#include <gtsam/nonlinear/summarization.h>
//
//// Uses partial QR approach by default
// gtsam::GaussianFactorGraph summarize(
//    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
//    const gtsam::KeySet& saved_keys);
//
// gtsam::NonlinearFactorGraph summarizeAsNonlinearContainer(
//    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
//    const gtsam::KeySet& saved_keys);

//*************************************************************************
// Nonlinear optimizers
//*************************************************************************
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
virtual class NonlinearOptimizerParams {
  NonlinearOptimizerParams();
  void print(string s = "") const;

  int getMaxIterations() const;
  double getRelativeErrorTol() const;
  double getAbsoluteErrorTol() const;
  double getErrorTol() const;
  string getVerbosity() const;

  void setMaxIterations(int value);
  void setRelativeErrorTol(double value);
  void setAbsoluteErrorTol(double value);
  void setErrorTol(double value);
  void setVerbosity(string s);

  string getLinearSolverType() const;
  void setLinearSolverType(string solver);

  void setIterativeParams(gtsam::IterativeOptimizationParameters* params);
  void setOrdering(const gtsam::Ordering& ordering);
  string getOrderingType() const;
  void setOrderingType(string ordering);

  bool isMultifrontal() const;
  bool isSequential() const;
  bool isCholmod() const;
  bool isIterative() const;

  // This only applies to python since matlab does not have lambda machinery.
  gtsam::NonlinearOptimizerParams::IterationHook iterationHook;
};

bool checkConvergence(double relativeErrorTreshold,
                      double absoluteErrorTreshold, double errorThreshold,
                      double currentError, double newError);
bool checkConvergence(const gtsam::NonlinearOptimizerParams& params,
                      double currentError, double newError);

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
virtual class GaussNewtonParams : gtsam::NonlinearOptimizerParams {
  GaussNewtonParams();
};

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
virtual class LevenbergMarquardtParams : gtsam::NonlinearOptimizerParams {
  LevenbergMarquardtParams();

  bool getDiagonalDamping() const;
  double getlambdaFactor() const;
  double getlambdaInitial() const;
  double getlambdaLowerBound() const;
  double getlambdaUpperBound() const;
  bool getUseFixedLambdaFactor();
  string getLogFile() const;
  string getVerbosityLM() const;

  void setDiagonalDamping(bool flag);
  void setlambdaFactor(double value);
  void setlambdaInitial(double value);
  void setlambdaLowerBound(double value);
  void setlambdaUpperBound(double value);
  void setUseFixedLambdaFactor(bool flag);
  void setLogFile(string s);
  void setVerbosityLM(string s);

  static gtsam::LevenbergMarquardtParams LegacyDefaults();
  static gtsam::LevenbergMarquardtParams CeresDefaults();

  static gtsam::LevenbergMarquardtParams EnsureHasOrdering(
      gtsam::LevenbergMarquardtParams params,
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::LevenbergMarquardtParams ReplaceOrdering(
      gtsam::LevenbergMarquardtParams params, const gtsam::Ordering& ordering);
};

#include <gtsam/nonlinear/DoglegOptimizer.h>
virtual class DoglegParams : gtsam::NonlinearOptimizerParams {
  DoglegParams();

  double getDeltaInitial() const;
  string getVerbosityDL() const;

  void setDeltaInitial(double deltaInitial) const;
  void setVerbosityDL(string verbosityDL) const;
};

#include <gtsam/nonlinear/GncParams.h>
enum GncLossType {
  GM /*Geman McClure*/,
  TLS /*Truncated least squares*/
};

template<PARAMS>
virtual class GncParams {
  GncParams(const PARAMS& baseOptimizerParams);
  GncParams();
  PARAMS baseOptimizerParams;
  gtsam::GncLossType lossType;
  size_t maxIterations;
  double muStep;
  double relativeCostTol;
  double weightsTol;
  gtsam::This::Verbosity verbosity;
  gtsam::KeyVector knownInliers;
  gtsam::KeyVector knownOutliers;

  void setLossType(const gtsam::GncLossType type);
  void setMaxIterations(const size_t maxIter);
  void setMuStep(const double step);
  void setRelativeCostTol(double value);
  void setWeightsTol(double value);
  void setVerbosityGNC(const gtsam::This::Verbosity value);
  void setKnownInliers(const gtsam::KeyVector& knownIn);
  void setKnownOutliers(const gtsam::KeyVector& knownOut);
  void print(const string& str = "GncParams: ") const;
  
  enum Verbosity {
    SILENT,
    SUMMARY,
    VALUES
  };
};

typedef gtsam::GncParams<gtsam::GaussNewtonParams> GncGaussNewtonParams;
typedef gtsam::GncParams<gtsam::LevenbergMarquardtParams> GncLMParams;
  
#include <gtsam/nonlinear/NonlinearOptimizer.h>
virtual class NonlinearOptimizer {
  gtsam::Values optimize();
  gtsam::Values optimizeSafely();
  double error() const;
  int iterations() const;
  gtsam::Values values() const;
  gtsam::NonlinearFactorGraph graph() const;
  gtsam::GaussianFactorGraph* iterate() const;
};

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
virtual class GaussNewtonOptimizer : gtsam::NonlinearOptimizer {
  GaussNewtonOptimizer(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Values& initialValues);
  GaussNewtonOptimizer(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Values& initialValues,
                       const gtsam::GaussNewtonParams& params);
};

#include <gtsam/nonlinear/DoglegOptimizer.h>
virtual class DoglegOptimizer : gtsam::NonlinearOptimizer {
  DoglegOptimizer(const gtsam::NonlinearFactorGraph& graph,
                  const gtsam::Values& initialValues);
  DoglegOptimizer(const gtsam::NonlinearFactorGraph& graph,
                  const gtsam::Values& initialValues,
                  const gtsam::DoglegParams& params);
  double getDelta() const;
};
  
#include <gtsam/nonlinear/GncOptimizer.h>
template<PARAMS>
virtual class GncOptimizer {
  GncOptimizer(const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& initialValues,
               const PARAMS& params);
  void setInlierCostThresholds(const double inth);
  const Vector& getInlierCostThresholds();
  void setInlierCostThresholdsAtProbability(const double alpha);
  void setWeights(const Vector w);
  const Vector& getWeights();
  gtsam::Values optimize();
};

typedef gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> GncGaussNewtonOptimizer;
typedef gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> GncLMOptimizer;

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
  LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph,
                              const gtsam::Values& initialValues);
  LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph,
                              const gtsam::Values& initialValues,
                              const gtsam::LevenbergMarquardtParams& params);
  double lambda() const;
  void print(string s = "") const;
};

#include <gtsam/nonlinear/ISAM2.h>
class ISAM2GaussNewtonParams {
  ISAM2GaussNewtonParams();

  void print(string s = "") const;

  /** Getters and Setters for all properties */
  double getWildfireThreshold() const;
  void setWildfireThreshold(double wildfireThreshold);
};

class ISAM2DoglegParams {
  ISAM2DoglegParams();

  void print(string s = "") const;

  /** Getters and Setters for all properties */
  double getWildfireThreshold() const;
  void setWildfireThreshold(double wildfireThreshold);
  double getInitialDelta() const;
  void setInitialDelta(double initialDelta);
  string getAdaptationMode() const;
  void setAdaptationMode(string adaptationMode);
  bool isVerbose() const;
  void setVerbose(bool verbose);
};

class ISAM2ThresholdMapValue {
  ISAM2ThresholdMapValue(char c, Vector thresholds);
  ISAM2ThresholdMapValue(const gtsam::ISAM2ThresholdMapValue& other);
};

class ISAM2ThresholdMap {
  ISAM2ThresholdMap();
  ISAM2ThresholdMap(const gtsam::ISAM2ThresholdMap& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(const gtsam::ISAM2ThresholdMapValue& value) const;
};

class ISAM2Params {
  ISAM2Params();

  void print(string s = "") const;

  /** Getters and Setters for all properties */
  void setOptimizationParams(
      const gtsam::ISAM2GaussNewtonParams& gauss_newton__params);
  void setOptimizationParams(const gtsam::ISAM2DoglegParams& dogleg_params);
  void setRelinearizeThreshold(double threshold);
  void setRelinearizeThreshold(const gtsam::ISAM2ThresholdMap& threshold_map);
  string getFactorization() const;
  void setFactorization(string factorization);

  int relinearizeSkip;
  bool enableRelinearization;
  bool evaluateNonlinearError;
  bool cacheLinearizedFactors;
  bool enableDetailedResults;
  bool enablePartialRelinearizationCheck;
  bool findUnusedFactorSlots;

  enum Factorization { CHOLESKY, QR };
  gtsam::ISAM2Params::Factorization factorization;
};

class ISAM2Clique {
  // Constructors
  ISAM2Clique();

  // Standard Interface
  Vector gradientContribution() const;
  void print(string s = "",
             gtsam::KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter);
};

class ISAM2Result {
  ISAM2Result();

  void print(string s = "") const;

  /** Getters and Setters for all properties */
  size_t getVariablesRelinearized() const;
  size_t getVariablesReeliminated() const;
  gtsam::FactorIndices getNewFactorsIndices() const;
  size_t getCliques() const;
  double getErrorBefore() const;
  double getErrorAfter() const;
};

class ISAM2 {
  ISAM2();
  ISAM2(const gtsam::ISAM2Params& params);
  ISAM2(const gtsam::ISAM2& other);

  bool equals(const gtsam::ISAM2& other, double tol) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  void printStats() const;
  void saveGraph(string s) const;

  gtsam::ISAM2Result update();
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            const gtsam::KeyGroupMap& constrainedKeys);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            gtsam::KeyGroupMap& constrainedKeys,
                            const gtsam::KeyList& noRelinKeys);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            gtsam::KeyGroupMap& constrainedKeys,
                            const gtsam::KeyList& noRelinKeys,
                            const gtsam::KeyList& extraReelimKeys);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            gtsam::KeyGroupMap& constrainedKeys,
                            const gtsam::KeyList& noRelinKeys,
                            const gtsam::KeyList& extraReelimKeys,
                            bool force_relinearize);

  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::ISAM2UpdateParams& updateParams);

  gtsam::Values getLinearizationPoint() const;
  bool valueExists(gtsam::Key key) const;
  gtsam::Values calculateEstimate() const;
  template <VALUE = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                     gtsam::Rot3, gtsam::Pose3, gtsam::Cal3_S2, gtsam::Cal3DS2,
                     gtsam::Cal3Bundler, gtsam::EssentialMatrix,
                     gtsam::PinholeCamera<gtsam::Cal3_S2>,
                     gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                     gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                     gtsam::PinholeCamera<gtsam::Cal3Unified>, Vector, Matrix}>
  VALUE calculateEstimate(size_t key) const;
  Matrix marginalCovariance(size_t key) const;
  gtsam::Values calculateBestEstimate() const;
  gtsam::VectorValues getDelta() const;
  double error(const gtsam::VectorValues& x) const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
  gtsam::VariableIndex getVariableIndex() const;
  const gtsam::KeySet& getFixedVariables() const;
  gtsam::ISAM2Params params() const;

  void printStats() const;
  gtsam::VectorValues gradientAtZero() const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/nonlinear/NonlinearISAM.h>
class NonlinearISAM {
  NonlinearISAM();
  NonlinearISAM(int reorderInterval);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  void printStats() const;
  void saveGraph(string s) const;
  gtsam::Values estimate() const;
  Matrix marginalCovariance(size_t key) const;
  int reorderInterval() const;
  int reorderCounter() const;
  void update(const gtsam::NonlinearFactorGraph& newFactors,
              const gtsam::Values& initialValues);
  void reorder_relinearize();

  // These might be expensive as instead of a reference the wrapper will make a
  // copy
  gtsam::GaussianISAM bayesTree() const;
  gtsam::Values getLinearizationPoint() const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
};

//*************************************************************************
// Nonlinear factor types
//*************************************************************************
#include <gtsam/nonlinear/PriorFactor.h>
template <T = {double,
               Vector,
               gtsam::Point2,
               gtsam::StereoPoint2,
               gtsam::Point3,
               gtsam::Rot2,
               gtsam::SO3,
               gtsam::SO4,
               gtsam::SOn,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Pose3,
               gtsam::Unit3,
               gtsam::Cal3_S2,
               gtsam::Cal3DS2,
               gtsam::Cal3Bundler,
               gtsam::Cal3Fisheye,
               gtsam::Cal3Unified,
               gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::imuBias::ConstantBias}>
virtual class PriorFactor : gtsam::NoiseModelFactor {
  PriorFactor(size_t key, const T& prior,
              const gtsam::noiseModel::Base* noiseModel);
  T prior() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/NonlinearEquality.h>
template <T = {gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2,
               gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose2,
               gtsam::Pose3, gtsam::Cal3_S2, gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality : gtsam::NoiseModelFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(size_t j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(size_t j, const T& feasible, double error_gain);

  // enabling serialization functionality
  void serialize() const;
};

template <T = {gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2,
               gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose2,
               gtsam::Pose3, gtsam::Cal3_S2, gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality2 : gtsam::NoiseModelFactor {
  NonlinearEquality2(gtsam::Key key1, gtsam::Key key2, double mu = 1e4);
  gtsam::Vector evaluateError(const T& x1, const T& x2);
};

}  // namespace gtsam
