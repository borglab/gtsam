//*************************************************************************
// certifiable
//*************************************************************************
namespace gtsam {

#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
class RiemannianStaircaseParams {
  enum class VerificationMethod { Spectra, DenseEigen };

  RiemannianStaircaseParams();

  // Wrapper-facing integer scalars use int; the C++ fields remain size_t.
  int pMin;
  int pMax;
  double alpha;
  gtsam::RiemannianStaircaseParams::VerificationMethod verificationMethod;
  double eta;
  int maxSpectraIters;
  int numLanczosVectors;
  double spectraTol;
  bool verbose;

  gtsam::AugmentedLagrangianParams getAlmParams() const;
  void setAlmParams(const gtsam::AugmentedLagrangianParams& parameters);
};

class RiemannianStaircaseResult {
  gtsam::Values values;
  int finalRank;
  bool certified;
  double minEigenvalue;
  double totalTime;

  bool hasRoundedSolution() const;
  gtsam::Values roundedValues() const;
  gtsam::Vector getRanksVisited() const;
  gtsam::Vector getCostPerLevel() const;
  gtsam::Vector getMinEigenvaluePerLevel() const;
  gtsam::Vector getQcqpBuildTimePerLevel() const;
  gtsam::Vector getNlpTimePerLevel() const;
  gtsam::Vector getVerifyTimePerLevel() const;
};

class RiemannianStaircaseOptimizer {
  RiemannianStaircaseOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::RiemannianStaircaseParams& params =
          gtsam::RiemannianStaircaseParams());

  gtsam::RiemannianStaircaseResult optimize() const;
  static gtsam::Values padInitialValues(const gtsam::Values& Y, int pMin);

  const gtsam::NonlinearFactorGraph& graph() const;
  const gtsam::Values& initialValues() const;
  const gtsam::RiemannianStaircaseParams& params() const;
};

}  // namespace gtsam
