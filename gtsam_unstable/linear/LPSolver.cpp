/*
 * LPSolver.cpp
 * @brief:
 * @date: May 1, 2014
 * @author: Duy-Nguyen Ta
 */

#include <gtsam_unstable/linear/LPSolver.h>
#include <gtsam/inference/Symbol.h>

#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using namespace gtsam;

namespace gtsam {

/* ************************************************************************* */
void LPSolver::buildMetaInformation() {
  size_t firstVarIndex = 1; // Warning: lpsolve's column number starts from 1 !!!
  // Collect variables in objective function first
  BOOST_FOREACH(Key key, objectiveCoeffs_ | boost::adaptors::map_keys) {
    variableColumnNo_.insert(make_pair(key, firstVarIndex));
    variableDims_.insert(make_pair(key, objectiveCoeffs_.dim(key)));
    firstVarIndex += variableDims_[key];
    freeVars_.insert(key);
  }
  // Now collect remaining keys in constraints
  VariableIndex factorIndex(*equalities_);
  BOOST_FOREACH(Key key, factorIndex | boost::adaptors::map_keys) {
    if (!variableColumnNo_.count(key)) {
      LinearEquality::shared_ptr factor = equalities_->at(*factorIndex[key].begin());
      size_t dim = factor->getDim(factor->find(key));
      variableColumnNo_.insert(make_pair(key, firstVarIndex));
      variableDims_.insert(make_pair(key, dim));
      firstVarIndex += variableDims_[key];
      freeVars_.insert(key);
    }
  }

  VariableIndex factorIndex2(*inequalities_);
  BOOST_FOREACH(Key key, factorIndex2 | boost::adaptors::map_keys) {
    if (!variableColumnNo_.count(key)) {
      LinearInequality::shared_ptr factor = inequalities_->at(*factorIndex2[key].begin());
      size_t dim = factor->getDim(factor->find(key));
      variableColumnNo_.insert(make_pair(key, firstVarIndex));
      variableDims_.insert(make_pair(key, dim));
      firstVarIndex += variableDims_[key];
      freeVars_.insert(key);
    }
  }

  // Collect the remaining keys in lowerBounds
  BOOST_FOREACH(Key key, lowerBounds_ | boost::adaptors::map_keys) {
    if (!variableColumnNo_.count(key)) {
      variableColumnNo_.insert(make_pair(key, firstVarIndex));
      variableDims_.insert(make_pair(key, lowerBounds_.dim(key)));
      firstVarIndex += variableDims_[key];
    }
    freeVars_.erase(key);
  }
  // Collect the remaining keys in upperBounds
  BOOST_FOREACH(Key key, upperBounds_ | boost::adaptors::map_keys) {
    if (!variableColumnNo_.count(key)) {
      variableColumnNo_.insert(make_pair(key, firstVarIndex));
      variableDims_.insert(make_pair(key, upperBounds_.dim(key)));
      firstVarIndex += variableDims_[key];
    }
    freeVars_.erase(key);
  }

  nrColumns_ = firstVarIndex - 1;
}

/* ************************************************************************* */
void LPSolver::addConstraints(const boost::shared_ptr<lprec>& lp,
    const JacobianFactor::shared_ptr& jacobian, int constraintType) const {
  if (!jacobian || !jacobian->isConstrained())
    throw runtime_error("LP only accepts constrained factors!");

  // Build column number from keys
  KeyVector keys = jacobian->keys();
  vector<int> columnNo = buildColumnNo(keys);

  // Add each row to lp one by one. TODO: is there a faster way?
  Matrix A = jacobian->getA();
  Vector b = jacobian->getb();
  for (int i = 0; i < A.rows(); ++i) {
    // A.row(i).data() gives wrong result so have to make a copy
    // TODO: Why? Probably because lpsolve's add_constraintex modifies this raw buffer!!!
    Vector r = A.row(i);

    // WARNING: lpsolve's add_constraintex modifies the columnNo raw buffer!
    // so we have to make a new copy for every row!!!!!
    vector<int> columnNoCopy(columnNo);

    if (!add_constraintex(lp.get(), columnNoCopy.size(), r.data(),
        columnNoCopy.data(), constraintType, b[i]))
      throw runtime_error("LP can't accept Gaussian noise!");
  }
}

/* ************************************************************************* */
void LPSolver::addBounds(const boost::shared_ptr<lprec>& lp) const {
  // Set lower bounds
  BOOST_FOREACH(Key key, lowerBounds_ | boost::adaptors::map_keys) {
    Vector lb = lowerBounds_.at(key);
    for (size_t i = 0; i < lb.size(); ++i) {
      set_lowbo(lp.get(), variableColumnNo_.at(key) + i, lb[i]);
    }
  }

  // Set upper bounds
  BOOST_FOREACH(Key key, upperBounds_ | boost::adaptors::map_keys) {
    Vector ub = upperBounds_.at(key);
    for (size_t i = 0; i < ub.size(); ++i) {
      set_upbo(lp.get(), variableColumnNo_.at(key) + i, ub[i]);
    }
  }

  // Set the rest as free variables
  BOOST_FOREACH(Key key, freeVars_) {
    for (size_t i = 0; i < variableDims_.at(key); ++i) {
      set_unbounded(lp.get(), variableColumnNo_.at(key) + i);
    }
  }
}

/* ************************************************************************* */
boost::shared_ptr<lprec> LPSolver::buildModel() const {
  boost::shared_ptr<lprec> lp(make_lp(0, nrColumns_));

  // Makes building the model faster if it is done rows by row
  set_add_rowmode(lp.get(), TRUE);

  // Add equality constraints
  BOOST_FOREACH(const LinearEquality::shared_ptr& factor, *equalities_) {
    addConstraints(lp, factor, EQ);
  }

  // Add inequality constraints
  BOOST_FOREACH(const LinearInequality::shared_ptr& factor, *inequalities_) {
    addConstraints(lp, factor, LE);
  }


  // Add bounds
  addBounds(lp);

  // Rowmode should be turned off again when done building the model
  set_add_rowmode(lp.get(), FALSE);

  // Finally, the objective function from the objective coefficients
  KeyVector keys;
  BOOST_FOREACH(Key key, objectiveCoeffs_ | boost::adaptors::map_keys) {
    keys.push_back(key);
  }

  Vector f = objectiveCoeffs_.vector(keys);
  vector<int> columnNo = buildColumnNo(keys);

  if (!set_obj_fnex(lp.get(), f.size(), f.data(), columnNo.data()))
    throw runtime_error("lpsolve cannot set obj function!");

  // Set the object direction to minimize
  set_minim(lp.get());

  // Set lp's verbose
  set_verbose(lp.get(), CRITICAL);

  return lp;
}

/* ************************************************************************* */
VectorValues LPSolver::convertToVectorValues(REAL* row) const {
  VectorValues values;
  BOOST_FOREACH(Key key, variableColumnNo_ | boost::adaptors::map_keys) {
    // Warning: the columnNo starts from 1, but C's array index starts from 0!!
    Vector v = Eigen::Map<Eigen::VectorXd>(&row[variableColumnNo_.at(key) - 1],
        variableDims_.at(key));
    values.insert(key, v);
  }
  return values;
}

/* ************************************************************************* */
VectorValues LPSolver::solve() const {
  static const bool debug = false;

  boost::shared_ptr<lprec> lp = buildModel();

  /* just out of curioucity, now show the model in lp format on screen */
  /* this only works if this is a console application. If not, use write_lp and a filename */
  if (debug)
    write_LP(lp.get(), stdout);

  int ret = ::solve(lp.get());
  if (ret != 0) {
    throw runtime_error(
        (boost::format(
            "lpsolve cannot find the optimal solution and terminates with %d error. "
                "See lpsolve's solve() documentation for details.") % ret).str());
  }
  REAL* row = NULL;
  get_ptr_variables(lp.get(), &row);

  VectorValues solution = convertToVectorValues(row);

  return solution;
}

} /* namespace gtsam */
