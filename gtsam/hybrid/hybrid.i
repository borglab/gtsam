//*************************************************************************
// hybrid
//*************************************************************************
namespace gtsam {

// #include <gtsam/inference/Key.h>
// class gtsam::KeyVector;

#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/slam/BetweenFactor.h>

virtual class DCFactor {};

#include <gtsam/hybrid/DCMixtureFactor.h>
template <T>
virtual class DCMixtureFactor : gtsam::DCFactor {
  DCMixtureFactor();
  DCMixtureFactor(const gtsam::KeyVector& keys,
                  const gtsam::DiscreteKeys& discreteKeys,
                  const std::vector<T*>& factors, bool normalized = false);
};

typedef gtsam::DCMixtureFactor<gtsam::BetweenFactor<double>>
    DCMixtureFactorBetweenFactorDouble;

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>

virtual class DCGaussianMixtureFactor : gtsam::DCFactor {
  DCGaussianMixtureFactor();
  DCGaussianMixtureFactor(
      const gtsam::KeyVector& continuousKeys,
      const gtsam::DiscreteKeys& discreteKeys,
      const gtsam::DCGaussianMixtureFactor::Factors& factors);
};

#include <gtsam/hybrid/GaussianMixture.h>

virtual class GaussianMixture : gtsam::DCGaussianMixtureFactor {
  GaussianMixture();
  GaussianMixture(size_t nrFrontals, const gtsam::KeyVector& continuousKeys,
                  const gtsam::DiscreteKeys& discreteKeys,
                  const gtsam::GaussianMixture::Conditionals& conditionals);
};

#include <gtsam/hybrid/DCFactorGraph.h>

class DCFactorGraph {
  DCFactorGraph();
  gtsam::DiscreteKeys discreteKeys() const;
};

#include <gtsam/hybrid/NonlinearHybridFactorGraph.h>

class NonlinearHybridFactorGraph {
  NonlinearHybridFactorGraph();
  NonlinearHybridFactorGraph(const gtsam::NonlinearFactorGraph& nonlinearGraph,
                             const gtsam::DiscreteFactorGraph& discreteGraph,
                             const gtsam::DCFactorGraph& dcGraph);
};

#include <gtsam/hybrid/GaussianHybridFactorGraph.h>

class GaussianHybridFactorGraph {
  GaussianHybridFactorGraph();
  GaussianHybridFactorGraph(const gtsam::GaussianFactorGraph& gaussianGraph,
                            const gtsam::DiscreteFactorGraph& discreteGraph,
                            const gtsam::DCFactorGraph& dcGraph);
};

#include <gtsam/hybrid/IncrementalHybrid.h>

class IncrementalHybrid {
  void update(gtsam::GaussianHybridFactorGraph graph,
              const gtsam::Ordering& ordering);
};

}  // namespace gtsam
