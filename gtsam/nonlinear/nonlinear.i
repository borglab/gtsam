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
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

class Symbol {
  Symbol();
  Symbol(char c, uint64_t j);
  Symbol(size_t key);

  size_t key() const;
  void print(const string& s = "") const;
  bool equals(const gtsam::Symbol& expected, double tol) const;

  char chr() const;
  uint64_t index() const;
  string string() const;
};

size_t symbol(char chr, size_t index);
char symbolChr(size_t key);
size_t symbolIndex(size_t key);

namespace symbol_shorthand {
size_t A(size_t j);
size_t B(size_t j);
size_t C(size_t j);
size_t D(size_t j);
size_t E(size_t j);
size_t F(size_t j);
size_t G(size_t j);
size_t H(size_t j);
size_t I(size_t j);
size_t J(size_t j);
size_t K(size_t j);
size_t L(size_t j);
size_t M(size_t j);
size_t N(size_t j);
size_t O(size_t j);
size_t P(size_t j);
size_t Q(size_t j);
size_t R(size_t j);
size_t S(size_t j);
size_t T(size_t j);
size_t U(size_t j);
size_t V(size_t j);
size_t W(size_t j);
size_t X(size_t j);
size_t Y(size_t j);
size_t Z(size_t j);
}  // namespace symbol_shorthand

// Default keyformatter
void PrintKeyList(
    const gtsam::KeyList& keys, const string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
void PrintKeyVector(
    const gtsam::KeyVector& keys, const string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
void PrintKeySet(
    const gtsam::KeySet& keys, const string& s = "",
    const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);

#include <gtsam/inference/LabeledSymbol.h>
class LabeledSymbol {
  LabeledSymbol(size_t full_key);
  LabeledSymbol(const gtsam::LabeledSymbol& key);
  LabeledSymbol(unsigned char valType, unsigned char label, size_t j);

  size_t key() const;
  unsigned char label() const;
  unsigned char chr() const;
  size_t index() const;

  gtsam::LabeledSymbol upper() const;
  gtsam::LabeledSymbol lower() const;
  gtsam::LabeledSymbol newChr(unsigned char c) const;
  gtsam::LabeledSymbol newLabel(unsigned char label) const;

  void print(string s = "") const;
};

size_t mrsymbol(unsigned char c, unsigned char label, size_t j);
unsigned char mrsymbolChr(size_t key);
unsigned char mrsymbolLabel(size_t key);
size_t mrsymbolIndex(size_t key);

#include <gtsam/inference/Ordering.h>
class Ordering {
  // Standard Constructors and Named Constructors
  Ordering();
  Ordering(const gtsam::Ordering& other);

  template <FACTOR_GRAPH = {gtsam::NonlinearFactorGraph,
                            gtsam::GaussianFactorGraph}>
  static gtsam::Ordering Colamd(const FACTOR_GRAPH& graph);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::Ordering& ord, double tol) const;

  // Standard interface
  size_t size() const;
  size_t at(size_t key) const;
  void push_back(size_t key);

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;
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

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;

  void saveGraph(const string& s) const;
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

#include <gtsam/nonlinear/CustomFactor.h>
virtual class CustomFactor : gtsam::NoiseModelFactor {
  /*
   * Note CustomFactor will not be wrapped for MATLAB, as there is no supporting
   * machinery there. This is achieved by adding `gtsam::CustomFactor` to the
   * ignore list in `matlab/CMakeLists.txt`.
   */
  CustomFactor();
  /*
   * Example:
   * ```
   * def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
   *    <calculated error>
   *    if not H is None:
   *        <calculate the Jacobian>
   *        H[0] = J1 # 2-d numpy array for a Jacobian block
   *        H[1] = J2
   *        ...
   *    return error # 1-d numpy array
   *
   * cf = CustomFactor(noise_model, keys, error_func)
   * ```
   */
  CustomFactor(const gtsam::SharedNoiseModel& noiseModel,
               const gtsam::KeyVector& keys,
               const gtsam::CustomErrorFunction& errorFunction);

  void print(string s = "",
             gtsam::KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter);
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
  void erase(size_t j);
  void swap(gtsam::Values& values);

  bool exists(size_t j) const;
  gtsam::KeyVector keys() const;

  gtsam::VectorValues zeroVectors() const;

  gtsam::Values retract(const gtsam::VectorValues& delta) const;
  gtsam::VectorValues localCoordinates(const gtsam::Values& cp) const;

  // enabling serialization functionality
  void serialize() const;

  // enable pickling in python
  void pickle() const;

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
  void insert(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert(size_t j, const gtsam::NavState& nav_state);
  void insert(size_t j, double c);

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
  void update(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void update(size_t j, const gtsam::NavState& nav_state);
  void update(size_t j, Vector vector);
  void update(size_t j, Matrix matrix);
  void update(size_t j, double c);

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
                 gtsam::imuBias::ConstantBias,
                 gtsam::NavState,
                 Vector,
                 Matrix,
                 double}>
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
  //  const boost::optional<Values>& linearizationPoint() const;

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
template<PARAMS>
virtual class GncParams {
  GncParams(const PARAMS& baseOptimizerParams);
  GncParams();
  void setVerbosityGNC(const This::Verbosity value);
  void print(const string& str) const;
  
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
  int getRelinearizeSkip() const;
  void setRelinearizeSkip(int relinearizeSkip);
  bool isEnableRelinearization() const;
  void setEnableRelinearization(bool enableRelinearization);
  bool isEvaluateNonlinearError() const;
  void setEvaluateNonlinearError(bool evaluateNonlinearError);
  string getFactorization() const;
  void setFactorization(string factorization);
  bool isCacheLinearizedFactors() const;
  void setCacheLinearizedFactors(bool cacheLinearizedFactors);
  bool isEnableDetailedResults() const;
  void setEnableDetailedResults(bool enableDetailedResults);
  bool isEnablePartialRelinearizationCheck() const;
  void setEnablePartialRelinearizationCheck(
      bool enablePartialRelinearizationCheck);
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

  // enable pickling in python
  void pickle() const;
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
  NonlinearEquality2(Key key1, Key key2, double mu = 1e4);
  gtsam::Vector evaluateError(const T& x1, const T& x2);
};

}  // namespace gtsam
