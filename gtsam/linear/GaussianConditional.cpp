/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianConditional.cpp
 * @brief  Conditional Gaussian Base class
 * @author Christian Potthast, Frank Dellaert
 */

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>

#include <boost/format.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <functional>
#include <list>
#include <string>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R,
    Key name1, const Matrix& S, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, name1, S, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R,
    Key name1, const Matrix& S, Key name2, const Matrix& T, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, name1, S, name2, T, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  void GaussianConditional::print(const string &s, const KeyFormatter& formatter) const {
    cout << s << "  Conditional density ";
    for (const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
      cout << (boost::format("[%1%]")%(formatter(*it))).str() << " ";
    }
    cout << endl;
    cout << formatMatrixIndented("  R = ", R()) << endl;
    for (const_iterator it = beginParents() ; it != endParents() ; ++it) {
      cout << formatMatrixIndented((boost::format("  S[%1%] = ")%(formatter(*it))).str(), getA(it))
        << endl;
    }
    cout << formatMatrixIndented("  d = ", getb(), true) << "\n";
    if (model_)
      model_->print("  Noise model: ");
    else
      cout << "  No noise model" << endl;
  }

  /* ************************************************************************* */
  bool GaussianConditional::equals(const GaussianFactor& f, double tol) const {
    if (const GaussianConditional* c = dynamic_cast<const GaussianConditional*>(&f)) {
      // check if the size of the parents_ map is the same
      if (parents().size() != c->parents().size())
        return false;

      // check if R_ and d_ are linear independent
      for (DenseIndex i = 0; i < Ab_.rows(); i++) {
        list<Vector> rows1, rows2;
        rows1.push_back(Vector(R().row(i)));
        rows2.push_back(Vector(c->R().row(i)));

        // check if the matrices are the same
        // iterate over the parents_ map
        for (const_iterator it = beginParents(); it != endParents(); ++it) {
          const_iterator it2 = c->beginParents() + (it - beginParents());
          if (*it != *(it2))
            return false;
          rows1.push_back(row(getA(it), i));
          rows2.push_back(row(c->getA(it2), i));
        }

        Vector row1 = concatVectors(rows1);
        Vector row2 = concatVectors(rows2);
        if (!linear_dependent(row1, row2, tol))
          return false;
      }

      // check if sigmas are equal
      if ((model_ && !c->model_) || (!model_ && c->model_)
        || (model_ && c->model_ && !model_->equals(*c->model_, tol)))
        return false;

      return true;
    } else {
      return false;
    }
  }

  /* ************************************************************************* */
  VectorValues GaussianConditional::solve(const VectorValues& x) const {
    // Concatenate all vector values that correspond to parent variables
    const Vector xS = x.vector(KeyVector(beginParents(), endParents()));

    // Update right-hand-side
    const Vector rhs = d() - S() * xS;

    // Solve matrix
    const Vector solution = R().triangularView<Eigen::Upper>().solve(rhs);

    // Check for indeterminant solution
    if (solution.hasNaN()) {
      throw IndeterminantLinearSystemException(keys().front());
    }

    // Insert solution into a VectorValues
    VectorValues result;
    DenseIndex vectorPosition = 0;
    for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      result.emplace(*frontal, solution.segment(vectorPosition, getDim(frontal)));
      vectorPosition += getDim(frontal);
    }

    return result;
  }

  /* ************************************************************************* */
  VectorValues GaussianConditional::solveOtherRHS(
    const VectorValues& parents, const VectorValues& rhs) const {
    // Concatenate all vector values that correspond to parent variables
    Vector xS = parents.vector(KeyVector(beginParents(), endParents()));

    // Instead of updating getb(), update the right-hand-side from the given rhs
    const Vector rhsR = rhs.vector(KeyVector(beginFrontals(), endFrontals()));
    xS = rhsR - S() * xS;

    // Solve Matrix
    Vector soln = R().triangularView<Eigen::Upper>().solve(xS);

    // Scale by sigmas
    if (model_)
      soln.array() *= model_->sigmas().array();

    // Insert solution into a VectorValues
    VectorValues result;
    DenseIndex vectorPosition = 0;
    for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      result.emplace(*frontal, soln.segment(vectorPosition, getDim(frontal)));
      vectorPosition += getDim(frontal);
    }

    return result;
  }

  /* ************************************************************************* */
  void GaussianConditional::solveTransposeInPlace(VectorValues& gy) const {
    Vector frontalVec = gy.vector(KeyVector(beginFrontals(), endFrontals()));
    frontalVec = R().transpose().triangularView<Eigen::Lower>().solve(frontalVec);

    // Check for indeterminant solution
    if (frontalVec.hasNaN()) throw IndeterminantLinearSystemException(this->keys().front());

    for (const_iterator it = beginParents(); it!= endParents(); it++)
      gy[*it].noalias() += -1.0 * getA(it).transpose() * frontalVec;

    // Scale by sigmas
    if (model_)
      frontalVec.array() *= model_->sigmas().array();

    // Write frontal solution into a VectorValues
    DenseIndex vectorPosition = 0;
    for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      gy[*frontal] = frontalVec.segment(vectorPosition, getDim(frontal));
      vectorPosition += getDim(frontal);
    }
  }

  /* ************************************************************************* */
  void GaussianConditional::scaleFrontalsBySigma(VectorValues& gy) const {
    DenseIndex vectorPosition = 0;
    for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      gy[*frontal].array() *= model_->sigmas().segment(vectorPosition, getDim(frontal)).array();
      vectorPosition += getDim(frontal);
    }
  }

}  // namespace gtsam
