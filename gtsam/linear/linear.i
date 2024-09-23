//*************************************************************************
// linear
//*************************************************************************
namespace gtsam {

namespace noiseModel {
#include <gtsam/linear/NoiseModel.h>
virtual class Base {
  void print(string s = "") const;
  // Methods below are available for all noise models. However, can't add them
  // because wrap (incorrectly) thinks robust classes derive from this Base as well.
  // bool isConstrained() const;
  // bool isUnit() const;
  // size_t dim() const;
  // gtsam::Vector sigmas() const;
};

virtual class Gaussian : gtsam::noiseModel::Base {
  static gtsam::noiseModel::Gaussian* Information(gtsam::Matrix R, bool smart = true);
  static gtsam::noiseModel::Gaussian* SqrtInformation(gtsam::Matrix R, bool smart = true);
  static gtsam::noiseModel::Gaussian* Covariance(gtsam::Matrix R, bool smart = true);

  bool equals(gtsam::noiseModel::Base& expected, double tol);

  // access to noise model
  gtsam::Matrix R() const;
  gtsam::Matrix information() const;
  gtsam::Matrix covariance() const;

  // Whitening operations
  gtsam::Vector whiten(gtsam::Vector v) const;
  gtsam::Vector unwhiten(gtsam::Vector v) const;
  gtsam::Matrix Whiten(gtsam::Matrix H) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Diagonal : gtsam::noiseModel::Gaussian {
  static gtsam::noiseModel::Diagonal* Sigmas(gtsam::Vector sigmas, bool smart = true);
  static gtsam::noiseModel::Diagonal* Variances(gtsam::Vector variances, bool smart = true);
  static gtsam::noiseModel::Diagonal* Precisions(gtsam::Vector precisions, bool smart = true);
  gtsam::Matrix R() const;

  // access to noise model
  gtsam::Vector sigmas() const;
  gtsam::Vector invsigmas() const;
  gtsam::Vector precisions() const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Constrained : gtsam::noiseModel::Diagonal {
    static gtsam::noiseModel::Constrained* MixedSigmas(gtsam::Vector mu, gtsam::Vector sigmas);
    static gtsam::noiseModel::Constrained* MixedSigmas(double m, gtsam::Vector sigmas);
    static gtsam::noiseModel::Constrained* MixedVariances(gtsam::Vector mu, gtsam::Vector variances);
    static gtsam::noiseModel::Constrained* MixedVariances(gtsam::Vector variances);
    static gtsam::noiseModel::Constrained* MixedPrecisions(gtsam::Vector mu, gtsam::Vector precisions);
    static gtsam::noiseModel::Constrained* MixedPrecisions(gtsam::Vector precisions);

    static gtsam::noiseModel::Constrained* All(size_t dim);
    static gtsam::noiseModel::Constrained* All(size_t dim, double mu);

    gtsam::noiseModel::Constrained* unit() const;

    // enabling serialization functionality
    void serializable() const;
};

virtual class Isotropic : gtsam::noiseModel::Diagonal {
  static gtsam::noiseModel::Isotropic* Sigma(size_t dim, double sigma, bool smart = true);
  static gtsam::noiseModel::Isotropic* Variance(size_t dim, double varianace, bool smart = true);
  static gtsam::noiseModel::Isotropic* Precision(size_t dim, double precision, bool smart = true);

  // access to noise model
  double sigma() const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Unit : gtsam::noiseModel::Isotropic {
  static gtsam::noiseModel::Unit* Create(size_t dim);

  // enabling serialization functionality
  void serializable() const;
};

namespace mEstimator {
virtual class Base {
  enum ReweightScheme { Scalar, Block };
  void print(string s = "") const;
};

virtual class Null: gtsam::noiseModel::mEstimator::Base {
  Null();
  static gtsam::noiseModel::mEstimator::Null* Create();

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Fair: gtsam::noiseModel::mEstimator::Base {
  Fair(double c);
  static gtsam::noiseModel::mEstimator::Fair* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Huber: gtsam::noiseModel::mEstimator::Base {
  Huber(double k);
  static gtsam::noiseModel::mEstimator::Huber* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Cauchy: gtsam::noiseModel::mEstimator::Base {
  Cauchy(double k);
  Cauchy(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::Cauchy* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Tukey: gtsam::noiseModel::mEstimator::Base {
  Tukey(double k);
  Tukey(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::Tukey* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Welsch: gtsam::noiseModel::mEstimator::Base {
  Welsch(double k);
  Welsch(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::Welsch* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class GemanMcClure: gtsam::noiseModel::mEstimator::Base {
  GemanMcClure(double c);
  GemanMcClure(double c, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::GemanMcClure* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class DCS: gtsam::noiseModel::mEstimator::Base {
  DCS(double c);
  DCS(double c, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::DCS* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class L2WithDeadZone: gtsam::noiseModel::mEstimator::Base {
  L2WithDeadZone(double k);
  L2WithDeadZone(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::L2WithDeadZone* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class AsymmetricTukey: gtsam::noiseModel::mEstimator::Base {
  AsymmetricTukey(double k);
  AsymmetricTukey(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::AsymmetricTukey* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class AsymmetricCauchy: gtsam::noiseModel::mEstimator::Base {
  AsymmetricCauchy(double k);
  AsymmetricCauchy(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::AsymmetricCauchy* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Custom: gtsam::noiseModel::mEstimator::Base {
  Custom(gtsam::noiseModel::mEstimator::CustomWeightFunction weight,
         gtsam::noiseModel::mEstimator::CustomLossFunction loss,
         gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight,
         std::string name);
  static gtsam::noiseModel::mEstimator::Custom* Create(
      gtsam::noiseModel::mEstimator::CustomWeightFunction weight,
      gtsam::noiseModel::mEstimator::CustomLossFunction loss,
      gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight,
      std::string name);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};


}///\namespace mEstimator

virtual class Robust : gtsam::noiseModel::Base {
  Robust(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  static gtsam::noiseModel::Robust* Create(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);

  // enabling serialization functionality
  void serializable() const;
};

}///\namespace noiseModel

#include <gtsam/linear/Sampler.h>
class Sampler {
  // Constructors
  Sampler(gtsam::noiseModel::Diagonal* model, int seed);
  Sampler(gtsam::Vector sigmas, int seed);

  // Standard Interface
  size_t dim() const;
  gtsam::Vector sigmas() const;
  gtsam::noiseModel::Diagonal* model() const;
  gtsam::Vector sample();
};

#include <gtsam/linear/VectorValues.h>
class VectorValues {
  //Constructors
  VectorValues();
  VectorValues(const gtsam::VectorValues& other);
  VectorValues(const gtsam::VectorValues& first, const gtsam::VectorValues& second);

  //Named Constructors
  static gtsam::VectorValues Zero(const gtsam::VectorValues& model);

  //Standard Interface
  size_t size() const;
  size_t dim(size_t j) const;
  bool exists(size_t j) const;
  void print(string s = "VectorValues",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::VectorValues& expected, double tol) const;
  void insert(size_t j, gtsam::Vector value);
  gtsam::Vector vector() const;
  gtsam::Vector vector(const gtsam::KeyVector& keys) const;
  gtsam::Vector at(size_t j) const;
  void insert(const gtsam::VectorValues& values);
  void update(const gtsam::VectorValues& values);

  //Advanced Interface
  void setZero();

  gtsam::VectorValues add(const gtsam::VectorValues& c) const;
  void addInPlace(const gtsam::VectorValues& c);
  gtsam::VectorValues subtract(const gtsam::VectorValues& c) const;
  gtsam::VectorValues scale(double a) const;
  void scaleInPlace(double a);

  bool hasSameStructure(const gtsam::VectorValues& other)  const;
  double dot(const gtsam::VectorValues& V) const;
  double norm() const;
  double squaredNorm() const;

  // enabling serialization functionality
  void serialize() const;
  string html() const;
};

#include <gtsam/linear/GaussianFactor.h>
virtual class GaussianFactor : gtsam::Factor {
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  double error(const gtsam::VectorValues& c) const;
  gtsam::GaussianFactor* clone() const;
  gtsam::GaussianFactor* negate() const;
  gtsam::Matrix augmentedInformation() const;
  gtsam::Matrix information() const;
  gtsam::Matrix augmentedJacobian() const;
  pair<gtsam::Matrix, gtsam::Vector> jacobian() const;
};

#include <gtsam/linear/JacobianFactor.h>
virtual class JacobianFactor : gtsam::GaussianFactor {
  //Constructors
  JacobianFactor();
  JacobianFactor(gtsam::Vector b_in);
  JacobianFactor(size_t i1, gtsam::Matrix A1, gtsam::Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, gtsam::Matrix A1, size_t i2, gtsam::Matrix A2, gtsam::Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, gtsam::Matrix A1, size_t i2, gtsam::Matrix A2, size_t i3, gtsam::Matrix A3,
      gtsam::Vector b, const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(const gtsam::GaussianFactorGraph& graph);
  JacobianFactor(const gtsam::GaussianFactorGraph& graph,
                 const gtsam::VariableSlots& p_variableSlots);
  JacobianFactor(const gtsam::GaussianFactorGraph& graph,
                 const gtsam::Ordering& ordering);
  JacobianFactor(const gtsam::GaussianFactorGraph& graph,
                 const gtsam::Ordering& ordering,
                 const gtsam::VariableSlots& p_variableSlots);
  JacobianFactor(const gtsam::GaussianFactor& factor);

  //Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  gtsam::Vector unweighted_error(const gtsam::VectorValues& c) const;
  gtsam::Vector error_vector(const gtsam::VectorValues& c) const;
  double error(const gtsam::VectorValues& c) const;

  //Standard Interface
  gtsam::Matrix getA() const;
  gtsam::Vector getb() const;
  size_t rows() const;
  size_t cols() const;
  bool isConstrained() const;
  pair<gtsam::Matrix, gtsam::Vector> jacobianUnweighted() const;
  gtsam::Matrix augmentedJacobianUnweighted() const;

  void transposeMultiplyAdd(double alpha, gtsam::Vector e, gtsam::VectorValues& x) const;
  gtsam::JacobianFactor whiten() const;

  pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> eliminate(const gtsam::Ordering& keys) const;

  void setModel(bool anyConstrained, gtsam::Vector sigmas);

  gtsam::noiseModel::Diagonal* get_model() const;

  // enabling serialization functionality
  void serialize() const;
};

pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> EliminateQR(
    const gtsam::GaussianFactorGraph& factors, const gtsam::Ordering& keys);

#include <gtsam/linear/HessianFactor.h>
virtual class HessianFactor : gtsam::GaussianFactor {
  //Constructors
  HessianFactor();
  HessianFactor(const gtsam::GaussianFactor& factor);
  HessianFactor(size_t j, gtsam::Matrix G, gtsam::Vector g, double f);
  HessianFactor(size_t j, gtsam::Vector mu, gtsam::Matrix Sigma);
  HessianFactor(size_t j1, size_t j2, gtsam::Matrix G11, gtsam::Matrix G12, gtsam::Vector g1, gtsam::Matrix G22,
      gtsam::Vector g2, double f);
  HessianFactor(size_t j1, size_t j2, size_t j3, gtsam::Matrix G11, gtsam::Matrix G12, gtsam::Matrix G13,
      gtsam::Vector g1, gtsam::Matrix G22, gtsam::Matrix G23, gtsam::Vector g2, gtsam::Matrix G33, gtsam::Vector g3,
      double f);
  HessianFactor(const gtsam::GaussianFactorGraph& factors);

  //Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  double error(const gtsam::VectorValues& c) const;

  //Standard Interface
  size_t rows() const;
  gtsam::Matrix information() const;
  double constantTerm() const;
  gtsam::Vector linearTerm() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/GaussianFactorGraph.h>
class GaussianFactorGraph {
  GaussianFactorGraph();
  GaussianFactorGraph(const gtsam::GaussianBayesNet& bayesNet);
  GaussianFactorGraph(const gtsam::GaussianBayesTree& bayesTree);

  // From FactorGraph
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianFactorGraph& lfgraph, double tol) const;
  size_t size() const;
  gtsam::GaussianFactor* at(size_t idx) const;
  gtsam::KeySet keys() const;
  gtsam::KeyVector keyVector() const;
  bool exists(size_t idx) const;

  // Building the graph
  void push_back(const gtsam::GaussianFactor* factor);
  void push_back(const gtsam::GaussianConditional* conditional);
  void push_back(const gtsam::GaussianFactorGraph& graph);
  void push_back(const gtsam::GaussianBayesNet& bayesNet);
  void push_back(const gtsam::GaussianBayesTree& bayesTree);
  void add(const gtsam::GaussianFactor& factor);
  void add(gtsam::Vector b);
  void add(size_t key1, gtsam::Matrix A1, gtsam::Vector b, const gtsam::noiseModel::Diagonal* model);
  void add(size_t key1, gtsam::Matrix A1, size_t key2, gtsam::Matrix A2, gtsam::Vector b,
      const gtsam::noiseModel::Diagonal* model);
  void add(size_t key1, gtsam::Matrix A1, size_t key2, gtsam::Matrix A2, size_t key3, gtsam::Matrix A3,
      gtsam::Vector b, const gtsam::noiseModel::Diagonal* model);

  // error and probability
  double error(const gtsam::VectorValues& c) const;
  double probPrime(const gtsam::VectorValues& c) const;
  void printErrors(const gtsam::VectorValues& c, string str = "GaussianFactorGraph: ", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const;

  gtsam::GaussianFactorGraph clone() const;
  gtsam::GaussianFactorGraph negate() const;

  // Optimizing and linear algebra
  gtsam::VectorValues optimize() const;
  gtsam::VectorValues optimizeDensely() const;
  gtsam::VectorValues optimize(const gtsam::Ordering& ordering) const;
  gtsam::VectorValues optimizeGradientSearch() const;
  gtsam::VectorValues gradient(const gtsam::VectorValues& x0) const;
  gtsam::VectorValues gradientAtZero() const;

  // Elimination and marginals
  gtsam::GaussianBayesNet* eliminateSequential();
  gtsam::GaussianBayesNet* eliminateSequential(gtsam::Ordering::OrderingType type);
  gtsam::GaussianBayesNet* eliminateSequential(const gtsam::Ordering& ordering);
  gtsam::GaussianBayesTree* eliminateMultifrontal();
  gtsam::GaussianBayesTree* eliminateMultifrontal(gtsam::Ordering::OrderingType type);
  gtsam::GaussianBayesTree* eliminateMultifrontal(const gtsam::Ordering& ordering);
  pair<gtsam::GaussianBayesNet*, gtsam::GaussianFactorGraph*> eliminatePartialSequential(
    const gtsam::Ordering& ordering);
  pair<gtsam::GaussianBayesNet*, gtsam::GaussianFactorGraph*> eliminatePartialSequential(
    const gtsam::KeyVector& keys);
  pair<gtsam::GaussianBayesTree*, gtsam::GaussianFactorGraph*> eliminatePartialMultifrontal(
    const gtsam::Ordering& ordering);
  pair<gtsam::GaussianBayesTree*, gtsam::GaussianFactorGraph*> eliminatePartialMultifrontal(
    const gtsam::KeyVector& keys);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::Ordering& ordering);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::KeyVector& key_vector);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::Ordering& ordering,
    const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::KeyVector& key_vector,
    const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::GaussianFactorGraph* marginal(const gtsam::KeyVector& key_vector);

  // Conversion to matrices
  gtsam::Matrix sparseJacobian_() const;
  gtsam::Matrix augmentedJacobian() const;
  gtsam::Matrix augmentedJacobian(const gtsam::Ordering& ordering) const;
  pair<gtsam::Matrix,gtsam::Vector> jacobian() const;
  pair<gtsam::Matrix,gtsam::Vector> jacobian(const gtsam::Ordering& ordering) const;
  gtsam::Matrix augmentedHessian() const;
  gtsam::Matrix augmentedHessian(const gtsam::Ordering& ordering) const;
  pair<gtsam::Matrix,gtsam::Vector> hessian() const;
  pair<gtsam::Matrix,gtsam::Vector> hessian(const gtsam::Ordering& ordering) const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/hybrid/HybridValues.h>
virtual class GaussianConditional : gtsam::JacobianFactor {
  // Constructors
  GaussianConditional(size_t key, gtsam::Vector d, gtsam::Matrix R,
                      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(size_t key, gtsam::Vector d, gtsam::Matrix R, size_t name1, gtsam::Matrix S,
                      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(size_t key, gtsam::Vector d, gtsam::Matrix R, size_t name1, gtsam::Matrix S,
                      size_t name2, gtsam::Matrix T,
                      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(const std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms,
                      size_t nrFrontals, gtsam::Vector d,
                      const gtsam::noiseModel::Diagonal* sigmas);

  // Constructors with no noise model
  GaussianConditional(size_t key, gtsam::Vector d, gtsam::Matrix R);
  GaussianConditional(size_t key, gtsam::Vector d, gtsam::Matrix R, size_t name1, gtsam::Matrix S);
  GaussianConditional(size_t key, gtsam::Vector d, gtsam::Matrix R, size_t name1, gtsam::Matrix S,
                      size_t name2, gtsam::Matrix T);
  GaussianConditional(const gtsam::KeyVector& keys, size_t nrFrontals,
                      const gtsam::VerticalBlockMatrix& augmentedMatrix);

  // Named constructors
  static gtsam::GaussianConditional FromMeanAndStddev(gtsam::Key key, 
                                                      const gtsam::Vector& mu,
                                                      double sigma);

  static gtsam::GaussianConditional FromMeanAndStddev(gtsam::Key key, 
                                                      const gtsam::Matrix& A,
                                                      gtsam::Key parent,
                                                      const gtsam::Vector& b,
                                                      double sigma);

  static gtsam::GaussianConditional FromMeanAndStddev(gtsam::Key key,
                                                      const gtsam::Matrix& A1,
                                                      gtsam::Key parent1, 
                                                      const gtsam::Matrix& A2,
                                                      gtsam::Key parent2, 
                                                      const gtsam::Vector& b,
                                                      double sigma);
  // Testable
  void print(string s = "GaussianConditional",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianConditional& cg, double tol) const;
  
  // Standard Interface
  double errorConstant() const;
  double logNormalizationConstant() const;
  double logProbability(const gtsam::VectorValues& x) const;
  double evaluate(const gtsam::VectorValues& x) const;
  double error(const gtsam::VectorValues& x) const;
  gtsam::Key firstFrontalKey() const;
  gtsam::VectorValues solve(const gtsam::VectorValues& parents) const;
  gtsam::JacobianFactor* likelihood(
      const gtsam::VectorValues& frontalValues) const;
  gtsam::JacobianFactor* likelihood(gtsam::Vector frontal) const;
  gtsam::VectorValues sample(const gtsam::VectorValues& parents) const;
  gtsam::VectorValues sample() const;
  
  // Advanced Interface
  gtsam::VectorValues solveOtherRHS(const gtsam::VectorValues& parents,
                                    const gtsam::VectorValues& rhs) const;
  void solveTransposeInPlace(gtsam::VectorValues& gy) const;
  gtsam::Matrix R() const;
  gtsam::Matrix S() const;
  gtsam::Vector d() const;

  // enabling serialization functionality
  void serialize() const;

  // Expose HybridValues versions
  double logProbability(const gtsam::HybridValues& x) const;
  double evaluate(const gtsam::HybridValues& x) const;
  double error(const gtsam::HybridValues& x) const;
};

#include <gtsam/linear/GaussianDensity.h>
virtual class GaussianDensity : gtsam::GaussianConditional {
  // Constructors
  GaussianDensity(gtsam::Key key, gtsam::Vector d, gtsam::Matrix R,
                  const gtsam::noiseModel::Diagonal* sigmas);

  static gtsam::GaussianDensity FromMeanAndStddev(gtsam::Key key,
                                                  const gtsam::Vector& mean,
                                                  double sigma);

  // Testable
  void print(string s = "GaussianDensity",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianDensity& cg, double tol) const;

  // Standard Interface
  gtsam::Vector mean() const;
  gtsam::Matrix covariance() const;
};

#include <gtsam/linear/GaussianBayesNet.h>
virtual class GaussianBayesNet {
    //Constructors
  GaussianBayesNet();
  GaussianBayesNet(const gtsam::GaussianConditional* conditional);

  // Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianBayesNet& other, double tol) const;
  size_t size() const;

  void push_back(gtsam::GaussianConditional* conditional);
  void push_back(const gtsam::GaussianBayesNet& bayesNet);
  gtsam::GaussianConditional* front() const;
  gtsam::GaussianConditional* back() const;

  // Standard interface
  // Standard Interface
  double logProbability(const gtsam::VectorValues& x) const;
  double evaluate(const gtsam::VectorValues& x) const;
  double error(const gtsam::VectorValues& x) const;

  gtsam::VectorValues optimize() const;
  gtsam::VectorValues optimize(const gtsam::VectorValues& given) const;
  gtsam::VectorValues optimizeGradientSearch() const;
  
  gtsam::VectorValues sample(const gtsam::VectorValues& given) const;
  gtsam::VectorValues sample() const;
  gtsam::VectorValues backSubstitute(const gtsam::VectorValues& gx) const;
  gtsam::VectorValues backSubstituteTranspose(const gtsam::VectorValues& gx) const;

  // FactorGraph derived interface
  gtsam::GaussianConditional* at(size_t idx) const;
  gtsam::KeySet keys() const;
  gtsam::KeyVector keyVector() const;
  bool exists(size_t idx) const;

  void saveGraph(const string& s) const;

  std::pair<gtsam::Matrix, gtsam::Vector> matrix() const; 
  gtsam::VectorValues gradient(const gtsam::VectorValues& x0) const;
  gtsam::VectorValues gradientAtZero() const;
  double error(const gtsam::VectorValues& x) const;
  double determinant() const;
  double logDeterminant() const;

  string dot(
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
  void saveGraph(
      string s,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::DotWriter& writer = gtsam::DotWriter()) const;
};

#include <gtsam/linear/GaussianBayesTree.h>
virtual class GaussianBayesTree {
  // Standard Constructors and Named Constructors
  GaussianBayesTree();
  GaussianBayesTree(const gtsam::GaussianBayesTree& other);
  bool equals(const gtsam::GaussianBayesTree& other, double tol) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter);
  size_t size() const;
  bool empty() const;
  size_t numCachedSeparatorMarginals() const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s,
                const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;

  gtsam::VectorValues optimize() const;
  gtsam::VectorValues optimizeGradientSearch() const;
  gtsam::VectorValues gradient(const gtsam::VectorValues& x0) const;
  gtsam::VectorValues gradientAtZero() const;
  double error(const gtsam::VectorValues& x) const;
  double determinant() const;
  double logDeterminant() const;
  gtsam::Matrix marginalCovariance(size_t key) const;
  gtsam::GaussianConditional* marginalFactor(size_t key) const;
  gtsam::GaussianFactorGraph* joint(size_t key1, size_t key2) const;
  gtsam::GaussianBayesNet* jointBayesNet(size_t key1, size_t key2) const;
};

#include <gtsam/linear/GaussianISAM.h>
class GaussianISAM {
  //Constructor
  GaussianISAM();

  //Standard Interface
  void update(const gtsam::GaussianFactorGraph& newFactors);
  void saveGraph(string s) const;
  void clear();
};

#include <gtsam/linear/IterativeSolver.h>
virtual class IterativeOptimizationParameters {
  string getVerbosity() const;
  void setVerbosity(string s) ;
};

//virtual class IterativeSolver {
//  IterativeSolver();
//  gtsam::VectorValues optimize ();
//};

#include <gtsam/linear/ConjugateGradientSolver.h>
virtual class ConjugateGradientParameters : gtsam::IterativeOptimizationParameters {
  ConjugateGradientParameters();
  int getMinIterations() const ;
  int getMaxIterations() const ;
  int getReset() const;
  double getEpsilon_rel() const;
  double getEpsilon_abs() const;

  void setMinIterations(int value);
  void setMaxIterations(int value);
  void setReset(int value);
  void setEpsilon_rel(double value);
  void setEpsilon_abs(double value);
};

#include <gtsam/linear/Preconditioner.h>
virtual class PreconditionerParameters {
  PreconditionerParameters();
};

virtual class DummyPreconditionerParameters : gtsam::PreconditionerParameters {
  DummyPreconditionerParameters();
};

virtual class BlockJacobiPreconditionerParameters : gtsam::PreconditionerParameters {
  BlockJacobiPreconditionerParameters();
};

#include <gtsam/linear/PCGSolver.h>
virtual class PCGSolverParameters : gtsam::ConjugateGradientParameters {
  PCGSolverParameters();
  void print(string s = "");
  void setPreconditionerParams(gtsam::PreconditionerParameters* preconditioner);
};

#include <gtsam/linear/SubgraphSolver.h>
virtual class SubgraphSolverParameters : gtsam::ConjugateGradientParameters {
  SubgraphSolverParameters();
};

virtual class SubgraphSolver  {
  SubgraphSolver(const gtsam::GaussianFactorGraph &A, const gtsam::SubgraphSolverParameters &parameters, const gtsam::Ordering& ordering);
  SubgraphSolver(const gtsam::GaussianFactorGraph &Ab1, const gtsam::GaussianFactorGraph& Ab2, const gtsam::SubgraphSolverParameters &parameters, const gtsam::Ordering& ordering);
  gtsam::VectorValues optimize() const;
};

#include <gtsam/linear/KalmanFilter.h>
class KalmanFilter {
  KalmanFilter(size_t n);
  // gtsam::GaussianDensity* init(gtsam::Vector x0, const gtsam::SharedDiagonal& P0);
  gtsam::GaussianDensity* init(gtsam::Vector x0, gtsam::Matrix P0);
  void print(string s = "") const;
  static size_t step(gtsam::GaussianDensity* p);
  gtsam::GaussianDensity* predict(gtsam::GaussianDensity* p, gtsam::Matrix F,
      gtsam::Matrix B, gtsam::Vector u, const gtsam::noiseModel::Diagonal* modelQ);
  gtsam::GaussianDensity* predictQ(gtsam::GaussianDensity* p, gtsam::Matrix F,
      gtsam::Matrix B, gtsam::Vector u, gtsam::Matrix Q);
  gtsam::GaussianDensity* predict2(gtsam::GaussianDensity* p, gtsam::Matrix A0,
      gtsam::Matrix A1, gtsam::Vector b, const gtsam::noiseModel::Diagonal* model);
  gtsam::GaussianDensity* update(gtsam::GaussianDensity* p, gtsam::Matrix H,
      gtsam::Vector z, const gtsam::noiseModel::Diagonal* model);
  gtsam::GaussianDensity* updateQ(gtsam::GaussianDensity* p, gtsam::Matrix H,
      gtsam::Vector z, gtsam::Matrix Q);
};

}
