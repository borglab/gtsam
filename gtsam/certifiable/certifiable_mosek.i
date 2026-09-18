//*************************************************************************
// MOSEK lifted SDP problems
//*************************************************************************
// MATLAB uses these container proxies; Python uses native lists and dictionaries.
namespace std {

#include <vector>
template<T>
class vector {
  vector();
  size_t size() const;
  T at(size_t pos) const;
  void push_back(const T& value);
};
typedef std::vector<double> vectordouble;

#include <map>
template<K, V>
class map {
  map();
  size_t size() const;
  double at(K key) const;
  void emplace(K key, double value);
};
typedef std::map<std::string, double> mapstringdouble;
typedef std::map<gtsam::Key, gtsam::DenseIndex> mapKeyDenseIndex;

}  // namespace std

namespace gtsam {

#include <gtsam/certifiable/LiftedSDPProblem.h>
class MosekMonolithicSDP {
  MosekMonolithicSDP(const gtsam::QcqpProblem& problem,
                     bool shareHomogeneousCoordinates = true);

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
                  gtsam::ChordalOrderingType orderingType,
                  bool shareHomogeneousCoordinates = true);

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
