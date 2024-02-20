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
  // Vector sigmas() const;
};

virtual class Gaussian : gtsam::noiseModel::Base {
  static gtsam::noiseModel::Gaussian* Information(Matrix R, bool smart = true);
  static gtsam::noiseModel::Gaussian* SqrtInformation(Matrix R, bool smart = true);
  static gtsam::noiseModel::Gaussian* Covariance(Matrix R, bool smart = true);

  bool equals(gtsam::noiseModel::Base& expected, double tol);

  // access to noise model
  Matrix R() const;
  Matrix information() const;
  Matrix covariance() const;

  // Whitening operations
  Vector whiten(Vector v) const;
  Vector unwhiten(Vector v) const;
  Matrix Whiten(Matrix H) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Diagonal : gtsam::noiseModel::Gaussian {
  static gtsam::noiseModel::Diagonal* Sigmas(Vector sigmas, bool smart = true);
  static gtsam::noiseModel::Diagonal* Variances(Vector variances, bool smart = true);
  static gtsam::noiseModel::Diagonal* Precisions(Vector precisions, bool smart = true);
  Matrix R() const;

  // access to noise model
  Vector sigmas() const;
  Vector invsigmas() const;
  Vector precisions() const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Constrained : gtsam::noiseModel::Diagonal {
    static gtsam::noiseModel::Constrained* MixedSigmas(Vector mu, Vector sigmas);
    static gtsam::noiseModel::Constrained* MixedSigmas(double m, Vector sigmas);
    static gtsam::noiseModel::Constrained* MixedVariances(Vector mu, Vector variances);
    static gtsam::noiseModel::Constrained* MixedVariances(Vector variances);
    static gtsam::noiseModel::Constrained* MixedPrecisions(Vector mu, Vector precisions);
    static gtsam::noiseModel::Constrained* MixedPrecisions(Vector precisions);

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
  static gtsam::noiseModel::mEstimator::Cauchy* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Tukey: gtsam::noiseModel::mEstimator::Base {
  Tukey(double k);
  static gtsam::noiseModel::mEstimator::Tukey* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Welsch: gtsam::noiseModel::mEstimator::Base {
  Welsch(double k);
  static gtsam::noiseModel::mEstimator::Welsch* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class GemanMcClure: gtsam::noiseModel::mEstimator::Base {
  GemanMcClure(double c);
  static gtsam::noiseModel::mEstimator::GemanMcClure* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class DCS: gtsam::noiseModel::mEstimator::Base {
  DCS(double c);
  static gtsam::noiseModel::mEstimator::DCS* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class L2WithDeadZone: gtsam::noiseModel::mEstimator::Base {
  L2WithDeadZone(double k);
  static gtsam::noiseModel::mEstimator::L2WithDeadZone* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class AsymmetricTukey: gtsam::noiseModel::mEstimator::Base {
  AsymmetricTukey(double k, gtsam::noiseModel::mEstimator::Base::ReweightScheme reweight);
  static gtsam::noiseModel::mEstimator::AsymmetricTukey* Create(double k);

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
  Sampler(Vector sigmas, int seed);

  // Standard Interface
  size_t dim() const;
  Vector sigmas() const;
  gtsam::noiseModel::Diagonal* model() const;
  Vector sample();
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
  void insert(size_t j, Vector value);
  Vector vector() const;
  Vector vector(const gtsam::KeyVector& keys) const;
  Vector at(size_t j) const;
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
  Matrix augmentedInformation() const;
  Matrix information() const;
  Matrix augmentedJacobian() const;
  pair<Matrix, Vector> jacobian() const;
};

#include <gtsam/linear/JacobianFactor.h>
virtual class JacobianFactor : gtsam::GaussianFactor {
  //Constructors
  JacobianFactor();
  JacobianFactor(Vector b_in);
  JacobianFactor(size_t i1, Matrix A1, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, size_t i3, Matrix A3,
      Vector b, const gtsam::noiseModel::Diagonal* model);
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
  Vector unweighted_error(const gtsam::VectorValues& c) const;
  Vector error_vector(const gtsam::VectorValues& c) const;
  double error(const gtsam::VectorValues& c) const;

  //Standard Interface
  Matrix getA() const;
  Vector getb() const;
  size_t rows() const;
  size_t cols() const;
  bool isConstrained() const;
  pair<Matrix, Vector> jacobianUnweighted() const;
  Matrix augmentedJacobianUnweighted() const;

  void transposeMultiplyAdd(double alpha, Vector e, gtsam::VectorValues& x) const;
  gtsam::JacobianFactor whiten() const;

  pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> eliminate(const gtsam::Ordering& keys) const;

  void setModel(bool anyConstrained, Vector sigmas);

  gtsam::noiseModel::Diagonal* get_model() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/HessianFactor.h>
virtual class HessianFactor : gtsam::GaussianFactor {
  //Constructors
  HessianFactor();
  HessianFactor(const gtsam::GaussianFactor& factor);
  HessianFactor(size_t j, Matrix G, Vector g, double f);
  HessianFactor(size_t j, Vector mu, Matrix Sigma);
  HessianFactor(size_t j1, size_t j2, Matrix G11, Matrix G12, Vector g1, Matrix G22,
      Vector g2, double f);
  HessianFactor(size_t j1, size_t j2, size_t j3, Matrix G11, Matrix G12, Matrix G13,
      Vector g1, Matrix G22, Matrix G23, Vector g2, Matrix G33, Vector g3,
      double f);
  HessianFactor(const gtsam::GaussianFactorGraph& factors);

  //Testable
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  double error(const gtsam::VectorValues& c) const;

  //Standard Interface
  size_t rows() const;
  Matrix information() const;
  double constantTerm() const;
  Vector linearTerm() const;

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
  void add(Vector b);
  void add(size_t key1, Matrix A1, Vector b, const gtsam::noiseModel::Diagonal* model);
  void add(size_t key1, Matrix A1, size_t key2, Matrix A2, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  void add(size_t key1, Matrix A1, size_t key2, Matrix A2, size_t key3, Matrix A3,
      Vector b, const gtsam::noiseModel::Diagonal* model);

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
  Matrix sparseJacobian_() const;
  Matrix augmentedJacobian() const;
  Matrix augmentedJacobian(const gtsam::Ordering& ordering) const;
  pair<Matrix,Vector> jacobian() const;
  pair<Matrix,Vector> jacobian(const gtsam::Ordering& ordering) const;
  Matrix augmentedHessian() const;
  Matrix augmentedHessian(const gtsam::Ordering& ordering) const;
  pair<Matrix,Vector> hessian() const;
  pair<Matrix,Vector> hessian(const gtsam::Ordering& ordering) const;

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
  GaussianConditional(size_t key, Vector d, Matrix R,
                      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
                      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
                      size_t name2, Matrix T,
                      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(const vector<std::pair<gtsam::Key, Matrix>> terms,
                      size_t nrFrontals, Vector d,
                      const gtsam::noiseModel::Diagonal* sigmas);

  // Constructors with no noise model
  GaussianConditional(size_t key, Vector d, Matrix R);
  GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S);
  GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
                      size_t name2, Matrix T);
  GaussianConditional(const gtsam::KeyVector& keys, size_t nrFrontals,
                      const gtsam::VerticalBlockMatrix& augmentedMatrix);

  // Named constructors
  static gtsam::GaussianConditional FromMeanAndStddev(gtsam::Key key, 
                                                      const Vector& mu,
                                                      double sigma);

  static gtsam::GaussianConditional FromMeanAndStddev(gtsam::Key key, 
                                                      const Matrix& A,
                                                      gtsam::Key parent,
                                                      const Vector& b,
                                                      double sigma);

  static gtsam::GaussianConditional FromMeanAndStddev(gtsam::Key key,
                                                      const Matrix& A1,
                                                      gtsam::Key parent1, 
                                                      const Matrix& A2,
                                                      gtsam::Key parent2, 
                                                      const Vector& b,
                                                      double sigma);
  // Testable
  void print(string s = "GaussianConditional",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianConditional& cg, double tol) const;
  
  // Standard Interface
  double logNormalizationConstant() const;
  double logProbability(const gtsam::VectorValues& x) const;
  double evaluate(const gtsam::VectorValues& x) const;
  double error(const gtsam::VectorValues& x) const;
  gtsam::Key firstFrontalKey() const;
  gtsam::VectorValues solve(const gtsam::VectorValues& parents) const;
  gtsam::JacobianFactor* likelihood(
      const gtsam::VectorValues& frontalValues) const;
  gtsam::JacobianFactor* likelihood(Vector frontal) const;
  gtsam::VectorValues sample(const gtsam::VectorValues& parents) const;
  gtsam::VectorValues sample() const;
  
  // Advanced Interface
  gtsam::VectorValues solveOtherRHS(const gtsam::VectorValues& parents,
                                    const gtsam::VectorValues& rhs) const;
  void solveTransposeInPlace(gtsam::VectorValues& gy) const;
  Matrix R() const;
  Matrix S() const;
  Vector d() const;

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
  GaussianDensity(gtsam::Key key, Vector d, Matrix R,
                  const gtsam::noiseModel::Diagonal* sigmas);

  static gtsam::GaussianDensity FromMeanAndStddev(gtsam::Key key,
                                                  const Vector& mean,
                                                  double sigma);

  // Testable
  void print(string s = "GaussianDensity",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::GaussianDensity& cg, double tol) const;

  // Standard Interface
  Vector mean() const;
  Matrix covariance() const;
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

  std::pair<Matrix, Vector> matrix() const; 
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
  Matrix marginalCovariance(size_t key) const;
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
  // gtsam::GaussianDensity* init(Vector x0, const gtsam::SharedDiagonal& P0);
  gtsam::GaussianDensity* init(Vector x0, Matrix P0);
  void print(string s = "") const;
  static size_t step(gtsam::GaussianDensity* p);
  gtsam::GaussianDensity* predict(gtsam::GaussianDensity* p, Matrix F,
      Matrix B, Vector u, const gtsam::noiseModel::Diagonal* modelQ);
  gtsam::GaussianDensity* predictQ(gtsam::GaussianDensity* p, Matrix F,
      Matrix B, Vector u, Matrix Q);
  gtsam::GaussianDensity* predict2(gtsam::GaussianDensity* p, Matrix A0,
      Matrix A1, Vector b, const gtsam::noiseModel::Diagonal* model);
  gtsam::GaussianDensity* update(gtsam::GaussianDensity* p, Matrix H,
      Vector z, const gtsam::noiseModel::Diagonal* model);
  gtsam::GaussianDensity* updateQ(gtsam::GaussianDensity* p, Matrix H,
      Vector z, Matrix Q);
};

}