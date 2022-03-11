//
// Created by Fan Jiang on 3/11/22.
//

#include <gtsam/hybrid/HybridDiscreteFactor.h>

#include <boost/make_shared.hpp>

namespace gtsam {

HybridDiscreteFactor::HybridDiscreteFactor(DiscreteFactor::shared_ptr other) {
  inner = other;
}

HybridDiscreteFactor::HybridDiscreteFactor(DecisionTreeFactor &&dtf) : inner(boost::make_shared<DecisionTreeFactor>(std::move(dtf))) {

};

}