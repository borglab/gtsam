/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GlobalConstraintSequentialSolver.cpp
 *
 * @brief Sequential elimination-based solver for GTSAM that can efficiently
 *        deal with global constraints
 *
 * @date Sept 2020
 * @author Gerry Chen
 * @author Frank Dellaert
 */

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GlobalConstraintSequentialSolver.h>
#include <gtsam/linear/JacobianGlobalConstraintFactor.h>
#include <gtsam/linear/VectorValues.h>

#include <boost/shared_ptr.hpp>
#include <vector>

using namespace std;

namespace gtsam {

shared_ptr<vector<VectorValues>> factorVector2VectorValues(
    const vector<JacobianGlobalConstraintFactor::shared_ptr> &gfg) {
  shared_ptr<vector<VectorValues>> ret = make_shared<vector<VectorValues>>();
  for (const JacobianGlobalConstraintFactor::shared_ptr &factor : gfg) {
    vector<VectorValues> factorAsVV(factor->rows());
    for (auto key_it = factor->begin(); key_it != factor->end(); ++key_it) {
      auto subA = factor->getA(key_it);
      for (size_t row = 0; row < factor->rows(); ++row)
        factorAsVV[row].insert(*key_it, subA.row(row));
    }
    ret->insert(ret->begin(), factorAsVV.begin(), factorAsVV.end());
  }
  return ret;
}

pair<shared_ptr<GaussianFactorGraph>,
     shared_ptr<vector<JacobianGlobalConstraintFactor::shared_ptr>>>
separateConstrained(const GaussianFactorGraph &gfg) {
  shared_ptr<GaussianFactorGraph> unconstrained =
      make_shared<GaussianFactorGraph>();
  shared_ptr<vector<JacobianGlobalConstraintFactor::shared_ptr>> constrained =
      make_shared<vector<JacobianGlobalConstraintFactor::shared_ptr>>();
  for (const boost::shared_ptr<GaussianFactor> &factor : gfg) {
    if (const JacobianGlobalConstraintFactor::shared_ptr &constraintFactor =
            boost::dynamic_pointer_cast<JacobianGlobalConstraintFactor>(factor))
      constrained->push_back(constraintFactor);
    else
      unconstrained->push_back(factor);
  }

  return make_pair(unconstrained, constrained);
}

VectorValues GlobalConstraintSequentialSolver::solve(
    const GaussianFactorGraph &gfg) {
  // separate global constraints out
  shared_ptr<GaussianFactorGraph> H;
  shared_ptr<vector<JacobianGlobalConstraintFactor::shared_ptr>>
      Agfg;                            // unconstrained gfg
  shared_ptr<vector<VectorValues>> A;  // constraint "matrix"
  tie(H, Agfg) = separateConstrained(gfg);
  A = factorVector2VectorValues(*Agfg);

  // solve unconstrained
  boost::shared_ptr<GaussianBayesNet> ubn;
  if (params_.ordering)
    ubn = H->eliminateSequential(*params_.ordering,
                                 params_.getEliminationFunction(), boost::none,
                                 params_.orderingType);
  else
    ubn = H->eliminateSequential(params_.getEliminationFunction(), boost::none,
                                 params_.orderingType);
  VectorValues xhat = ubn->optimize();

  if (A->size() == 0)
    return xhat;

  // calculate correction term...
  // calculate Qa as A = Qa @ R where R is ubn (HTH = RTR)
  vector<VectorValues> QaVV(A->size());
  transform(A->cbegin(), A->cend(), QaVV.begin(),
            [&ubn](VectorValues a) -> VectorValues {
              return ubn->backSubstituteTranspose(a);
            });
  // Qa2 = Qa @ QaTranspose
  Matrix Qa2(QaVV.size(), QaVV.size());
  for (size_t row = 0; row < QaVV.size(); ++row) {
    for (size_t col = row; col < QaVV.size(); ++col) {
      Qa2(row, col) = QaVV[row].dot(QaVV[col]);
      Qa2(col, row) = Qa2(row, col);
    }
  }
  // solve Qa.QaT.v = A.xhat - b for dual variable v
  FastList<Vector> errors;
  errors.resize(Agfg->size());
  transform(Agfg->begin(), Agfg->end(), errors.begin(),
            [&xhat](const JacobianGlobalConstraintFactor::shared_ptr &factor)
                -> Vector { return factor->unweighted_error(xhat); });
  Vector error = concatVectors(errors);
  Vector v = Qa2.colPivHouseholderQr().solve(error);  // dense solve
  // get QaT.v
  VectorValues QaTv;
  for (int i = 0; i < v.size(); ++i) {
    QaTv.addInPlace_(QaVV[i].scale(v[i]));
  }
  // solve R.dx = QaT.v for correction term dx
  VectorValues dx = ubn->backSubstitute(QaTv);

  return xhat - dx;

  throw std::exception();
}
}  // namespace gtsam