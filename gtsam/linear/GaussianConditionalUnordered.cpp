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
 * @author Christian Potthast
 */

#include <string.h>
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

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditionalUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  GaussianConditionalUnordered::GaussianConditionalUnordered(
    Key key, const Vector& d, const Matrix& R, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  GaussianConditionalUnordered::GaussianConditionalUnordered(
    Key key, const Vector& d, const Matrix& R,
    Key name1, const Matrix& S, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, name1, S, d, sigmas) {}

  /* ************************************************************************* */
  GaussianConditionalUnordered::GaussianConditionalUnordered(
    Key key, const Vector& d, const Matrix& R,
    Key name1, const Matrix& S, Key name2, const Matrix& T, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, name1, S, name2, T, d, sigmas) {}

  /* ************************************************************************* */
  void GaussianConditionalUnordered::print(const string &s, const IndexFormatter& formatter) const
  {
    cout << s << ": density on ";
    for(const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
      cout << (boost::format("[%1%]")%(formatter(*it))).str() << " ";
    }
    cout << endl;
    gtsam::print(Matrix(get_R()),"R");
    for(const_iterator it = beginParents() ; it != endParents() ; ++it ) {
      gtsam::print(Matrix(getA(it)), (boost::format("S[%1%]")%(formatter(*it))).str());
    }
    gtsam::print(Vector(getb()),"d");
    model_->print("sigmas");
  }    

  /* ************************************************************************* */
  bool GaussianConditionalUnordered::equals(const GaussianConditionalUnordered &c, double tol) const {
    // check if the size of the parents_ map is the same
    if (parents().size() != c.parents().size())
      return false;

    // check if R_ and d_ are linear independent
    for (size_t i=0; i<Ab_.rows(); i++) {
      list<Vector> rows1; rows1.push_back(Vector(get_R().row(i)));
      list<Vector> rows2; rows2.push_back(Vector(c.get_R().row(i)));

      // check if the matrices are the same
      // iterate over the parents_ map
      for (const_iterator it = beginParents(); it != endParents(); ++it) {
        const_iterator it2 = c.beginParents() + (it-beginParents());
        if(*it != *(it2))
          return false;
        rows1.push_back(row(getA(it), i));
        rows2.push_back(row(c.getA(it2), i));
      }

      Vector row1 = concatVectors(rows1);
      Vector row2 = concatVectors(rows2);
      if (!linear_dependent(row1, row2, tol))
        return false;
    }

    // check if sigmas are equal
    if ((model_ && !c.model_) || (!model_ && c.model_)
      || !model_->equals(*c.model_, tol))
      return false;

    return true;
  }

  /* ************************************************************************* */
  void GaussianConditionalUnordered::solveInPlace(VectorValuesUnordered& x) const {
    static const bool debug = false;
    if(debug) this->print("Solving conditional in place");
    Vector xS = internal::extractVectorValuesSlices(x, this->beginParents(), this->endParents());
    xS = this->getb() - this->get_S() * xS;
    Vector soln = this->get_R().triangularView<Eigen::Upper>().solve(xS);

    // Check for indeterminant solution
    if(soln.unaryExpr(!boost::lambda::bind(ptr_fun(isfinite<double>), boost::lambda::_1)).any())
      throw IndeterminantLinearSystemException(this->keys().front());

    if(debug) {
      gtsam::print(Matrix(this->get_R()), "Calling backSubstituteUpper on ");
      gtsam::print(soln, "full back-substitution solution: ");
    }
    internal::writeVectorValuesSlices(soln, x, this->beginFrontals(), this->endFrontals());
  }

  /* ************************************************************************* */
  void GaussianConditionalUnordered::solveTransposeInPlace(VectorValuesUnordered& gy) const {
    Vector frontalVec = internal::extractVectorValuesSlices(gy, beginFrontals(), endFrontals());
    frontalVec = gtsam::backSubstituteUpper(frontalVec,Matrix(get_R()));

    // Check for indeterminant solution
    if(frontalVec.unaryExpr(!boost::lambda::bind(ptr_fun(isfinite<double>), boost::lambda::_1)).any())
      throw IndeterminantLinearSystemException(this->keys().front());

    GaussianConditionalUnordered::const_iterator it;
    for (it = beginParents(); it!= endParents(); it++) {
      const Key i = *it;
      gtsam::transposeMultiplyAdd(-1.0, getA(it), frontalVec, gy[i]);
    }
    internal::writeVectorValuesSlices(frontalVec, gy, beginFrontals(), endFrontals());
  }

  /* ************************************************************************* */
  void GaussianConditionalUnordered::scaleFrontalsBySigma(VectorValuesUnordered& gy) const {
    Vector frontalVec = internal::extractVectorValuesSlices(gy, beginFrontals(), endFrontals());
    if(model_)
      frontalVec.array() *= model_->sigmas().array();
    internal::writeVectorValuesSlices(frontalVec, gy, beginFrontals(), endFrontals());
  }

}

