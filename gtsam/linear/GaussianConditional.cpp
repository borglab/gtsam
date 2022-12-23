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

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/linearExceptions.h>

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

// In Wrappers we have no access to this so have a default ready
static std::mt19937_64 kRandomNumberGenerator(42);

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************ */
  GaussianConditional::GaussianConditional(Key key, const Vector& d,
                                           const Matrix& R, Key parent1,
                                           const Matrix& S,
                                           const SharedDiagonal& sigmas)
      : BaseFactor(key, R, parent1, S, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************ */
  GaussianConditional::GaussianConditional(Key key, const Vector& d,
                                           const Matrix& R, Key parent1,
                                           const Matrix& S, Key parent2,
                                           const Matrix& T,
                                           const SharedDiagonal& sigmas)
      : BaseFactor(key, R, parent1, S, parent2, T, d, sigmas),
        BaseConditional(1) {}

  /* ************************************************************************ */
  GaussianConditional GaussianConditional::FromMeanAndStddev(
      Key key, const Matrix& A, Key parent, const Vector& b, double sigma) {
    // |Rx + Sy - d| = |x-(Ay + b)|/sigma
    const Matrix R = Matrix::Identity(b.size(), b.size());
    const Matrix S = -A;
    const Vector d = b;
    return GaussianConditional(key, d, R, parent, S,
                               noiseModel::Isotropic::Sigma(b.size(), sigma));
  }

  /* ************************************************************************ */
  GaussianConditional GaussianConditional::FromMeanAndStddev(
      Key key, const Matrix& A1, Key parent1, const Matrix& A2, Key parent2,
      const Vector& b, double sigma) {
    // |Rx + Sy + Tz - d| = |x-(A1 y + A2 z + b)|/sigma
    const Matrix R = Matrix::Identity(b.size(), b.size());
    const Matrix S = -A1;
    const Matrix T = -A2;
    const Vector d = b;
    return GaussianConditional(key, d, R, parent1, S, parent2, T,
                               noiseModel::Isotropic::Sigma(b.size(), sigma));
  }

  /* ************************************************************************ */
  void GaussianConditional::print(const string &s, const KeyFormatter& formatter) const {
    cout << s << " p(";
    for (const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
      cout << (boost::format("%1%") % (formatter(*it))).str()
           << (nrFrontals() > 1 ? " " : "");
    }

    if (nrParents()) {
      cout << " |";
      for (const_iterator it = beginParents(); it != endParents(); ++it) {
        cout << " " << (boost::format("%1%") % (formatter(*it))).str();
      }
    }
    cout << ")" << endl;

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
double GaussianConditional::logDeterminant() const {
  double logDet;
  if (this->get_model()) {
    Vector diag = this->R().diagonal();
    this->get_model()->whitenInPlace(diag);
    logDet = diag.unaryExpr([](double x) { return log(x); }).sum();
  } else {
    logDet =
        this->R().diagonal().unaryExpr([](double x) { return log(x); }).sum();
  }
  return logDet;
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

  /* ************************************************************************ */
  JacobianFactor::shared_ptr GaussianConditional::likelihood(
      const VectorValues& frontalValues) const {
    // Error is |Rx - (d - Sy - Tz - ...)|^2
    // so when we instantiate x (which has to be completely known) we beget:
    // |Sy + Tz + ... - (d - Rx)|^2
    // The noise model just transfers over!

    // Get frontalValues as vector
    const Vector x =
        frontalValues.vector(KeyVector(beginFrontals(), endFrontals()));

    // Copy the augmented Jacobian matrix:
    auto newAb = Ab_;

    // Restrict view to parent blocks
    newAb.firstBlock() += nrFrontals_;

    // Update right-hand-side (last column)
    auto last = newAb.matrix().cols() - 1;
    const auto RR = R().triangularView<Eigen::Upper>();
    newAb.matrix().col(last) -= RR * x;

    // The keys now do not include the frontal keys:
    KeyVector newKeys;
    newKeys.reserve(nrParents());
    for (auto&& key : parents()) newKeys.push_back(key);

    // Hopefully second newAb copy below is optimized out...
    return boost::make_shared<JacobianFactor>(newKeys, newAb, model_);
  }

  /* **************************************************************************/
  JacobianFactor::shared_ptr GaussianConditional::likelihood(
      const Vector& frontal) const {
    if (nrFrontals() != 1)
      throw std::invalid_argument(
          "GaussianConditional Single value likelihood can only be invoked on "
          "single-variable conditional");
    VectorValues values;
    values.insert(keys_[0], frontal);
    return likelihood(values);
  }

  /* ************************************************************************ */
  VectorValues GaussianConditional::sample(const VectorValues& parentsValues,
                                           std::mt19937_64* rng,
                                           const SharedDiagonal& model) const {
    if (nrFrontals() != 1) {
      throw std::invalid_argument(
          "GaussianConditional::sample can only be called on single variable "
          "conditionals");
    }

    VectorValues solution = solve(parentsValues);
    Key key = firstFrontalKey();

    Vector sigmas;
    if (model_) {
      sigmas = model_->sigmas();
    } else if (model) {
      sigmas = model->sigmas();
    } else {
      throw std::invalid_argument(
          "GaussianConditional::sample can only be called if a diagonal noise "
          "model was specified at construction.");
    }
    solution[key] += Sampler::sampleDiagonal(sigmas, rng);
    return solution;
  }

  VectorValues GaussianConditional::sample(std::mt19937_64* rng,
                                           const SharedDiagonal& model) const {
    if (nrParents() != 0)
      throw std::invalid_argument(
          "sample() can only be invoked on no-parent prior");
    VectorValues values;
    return sample(values, rng, model);
  }

  /* ************************************************************************ */
  VectorValues GaussianConditional::sample(const SharedDiagonal& model) const {
    return sample(&kRandomNumberGenerator, model);
  }

  VectorValues GaussianConditional::sample(const VectorValues& given,
                                           const SharedDiagonal& model) const {
    return sample(given, &kRandomNumberGenerator, model);
  }

  /* ************************************************************************ */
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  void GTSAM_DEPRECATED
  GaussianConditional::scaleFrontalsBySigma(VectorValues& gy) const {
    DenseIndex vectorPosition = 0;
    for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      gy[*frontal].array() *= model_->sigmas().segment(vectorPosition, getDim(frontal)).array();
      vectorPosition += getDim(frontal);
    }
  }
#endif

}  // namespace gtsam
