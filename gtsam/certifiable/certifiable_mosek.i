//*************************************************************************
// MOSEK lifted SDP problems
//*************************************************************************
namespace gtsam {

#include <gtsam/certifiable/LiftedSDPProblem.h>
class MosekMonolithicSDP {
  MosekMonolithicSDP(const gtsam::QcqpProblem& problem);

  bool solve(
      const std::map<std::string, double>& mosekParams =
          std::map<std::string, double>());
  double objectiveValue() const;
  std::string problemStatus() const;
  double solveTimeSeconds() const;

  gtsam::Values qcqpValues() const;
  std::vector<double> variableEVRs() const;

  const gtsam::KeyVector& orderedKeys() const;
  const std::map<gtsam::Key, gtsam::DenseIndex>& orderedKeyDims() const;
};

enum class ChordalOrderingType { Metis, Colamd };

class MosekChordalSDP {
  MosekChordalSDP(const gtsam::QcqpProblem& problem,
                  gtsam::ChordalOrderingType orderingType);

  bool solve(
      const std::map<std::string, double>& mosekParams =
          std::map<std::string, double>());
  double objectiveValue() const;
  std::string problemStatus() const;
  double solveTimeSeconds() const;

  gtsam::Values qcqpValues() const;
  std::vector<double> variableEVRs() const;

  const gtsam::KeyVector& orderedKeys() const;
  const std::map<gtsam::Key, gtsam::DenseIndex>& orderedKeyDims() const;
  const gtsam::SymbolicBayesTree& bayesTree() const;
};

}  // namespace gtsam
