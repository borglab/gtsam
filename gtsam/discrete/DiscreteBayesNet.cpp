/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteBayesNet.cpp
 * @date Feb 15, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>

#include <boost/make_shared.hpp>
#include <boost/range/adaptor/reversed.hpp>

namespace gtsam {

// Instantiate base class
template class FactorGraph<DiscreteConditional>;

/* ************************************************************************* */
bool DiscreteBayesNet::equals(const This& bn, double tol) const {
  return Base::equals(bn, tol);
}

/* ************************************************************************* */
double DiscreteBayesNet::logProbability(const DiscreteValues& values) const {
  // evaluate all conditionals and add
  double result = 0.0;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    result += conditional->logProbability(values);
  return result;
}

/* ************************************************************************* */
double DiscreteBayesNet::evaluate(const DiscreteValues& values) const {
  // evaluate all conditionals and multiply
  double result = 1.0;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    result *= (*conditional)(values);
  return result;
}

/* ************************************************************************* */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
DiscreteValues DiscreteBayesNet::optimize() const {
  DiscreteValues result;
  return optimize(result);
}

DiscreteValues DiscreteBayesNet::optimize(DiscreteValues result) const {
  // solve each node in turn in topological sort order (parents first)
#ifdef _MSC_VER
#pragma message("DiscreteBayesNet::optimize (deprecated) does not compute MPE!")
#else
#warning "DiscreteBayesNet::optimize (deprecated) does not compute MPE!"
#endif
  for (auto conditional : boost::adaptors::reverse(*this))
    conditional->solveInPlace(&result);
  return result;
}
#endif

/* ************************************************************************* */
DiscreteValues DiscreteBayesNet::sample() const {
  DiscreteValues result;
  return sample(result);
}

DiscreteValues DiscreteBayesNet::sample(DiscreteValues result) const {
  // sample each node in turn in topological sort order (parents first)
  for (auto conditional : boost::adaptors::reverse(*this))
    conditional->sampleInPlace(&result);
  return result;
}

/* *********************************************************************** */
std::string DiscreteBayesNet::markdown(
    const KeyFormatter& keyFormatter,
    const DiscreteFactor::Names& names) const {
  using std::endl;
  std::stringstream ss;
  ss << "`DiscreteBayesNet` of size " << size() << endl << endl;
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    ss << conditional->markdown(keyFormatter, names) << endl;
  return ss.str();
}

/* *********************************************************************** */
std::string DiscreteBayesNet::html(const KeyFormatter& keyFormatter,
                                   const DiscreteFactor::Names& names) const {
  using std::endl;
  std::stringstream ss;
  ss << "<div><p><tt>DiscreteBayesNet</tt> of size " << size() << "</p>";
  for (const DiscreteConditional::shared_ptr& conditional : *this)
    ss << conditional->html(keyFormatter, names) << endl;
  return ss.str();
}

/* ************************************************************************* */
}  // namespace gtsam
