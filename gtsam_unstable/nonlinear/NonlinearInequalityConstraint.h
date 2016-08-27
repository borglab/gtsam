/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file 	 NonlinearConstraint.h
 * @brief  Nonlinear inequalities allways assumed to be in the form c(X) <= 0
 * @author Duy-Nguyen Ta
 * @author Ivan Dario Jimenez
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam_unstable/nonlinear/NonlinearConstraint.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityConstraint.h>

namespace gtsam {
  
  class NonlinearInequalityConstraint: public NonlinearConstraint {
  public:
    template<typename CONTAINER>
    NonlinearInequalityConstraint(const SharedNoiseModel &noiseModel, const CONTAINER &keys, Key dualKey, bool isActive)
      : NonlinearConstraint(noiseModel, keys, dualKey, isActive) {}
    
    virtual const NonlinearConstraint & getEquality() const = 0;
  };
  
/* ************************************************************************* */
/** A convenient base class for creating a nonlinear equality constraint with 1
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE, typename ERROR_FUNCTOR>
class NonlinearInequalityConstraint1: public NonlinearInequalityConstraint {

public:

  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:

  typedef NonlinearInequalityConstraint1<VALUE, ERROR_FUNCTOR> This;

private:
  static const int X1Dim = traits < VALUE > ::dimension;

public:

  typedef NonlinearEqualityConstraint1<VALUE, ERROR_FUNCTOR> ThisEquality;

  const ThisEquality equality_;

  inline Key key() {
    return keys_[0];
  }

  /**
   * Constructor
   * @param j key of the variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequalityConstraint1(Key key, Key dualKey) :
      NonlinearInequalityConstraint(noiseModel::Constrained::All(1),
          cref_list_of < 1 > (key), dualKey, false), equality_(key, dualKey) {
  }

  virtual ~NonlinearInequalityConstraint1() {
  }
  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector unwhitenedError(const Values &x,
      boost::optional<std::vector<Matrix> &> H = boost::none) const override {
    if (this->active(x)) {
      ERROR_FUNCTOR f;
      Vector signedError;
      if (H) {
        signedError = f(x.at < X > (keys_[0]), (*H)[0]);
      } else {
        signedError = f(x.at < X > (keys_[0]));
      }

      //Return no error if we are on the right side of the inequality
      for (int index = 0; index < signedError.size(); index++) {
        signedError[index] = signedError[index] > 0 ? signedError[index] : 0;
      }

      return signedError;
    } else {
      return Vector::Zero(this->dim());
    }
  }

  virtual GaussianFactor::shared_ptr multipliedHessian(const Values &x,
      const VectorValues &duals) const override {
    return equality_.multipliedHessian(x, duals);
  }

  virtual void evaluateHessians(const X& x1, std::vector<Matrix>& G11) const {
    equality_.evaluateHessians(x1, G11);
  }

  virtual const NonlinearConstraint& getEquality() const {
    return equality_;
  }
  
private:

  /** Serialization function */
  friend class boost::serialization::access;

  template<class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NonlinearIneqality1",
            boost::serialization::base_object < Base > (*this));
  }
};
// \class NonlinearEqualityConstraint1

/* ************************************************************************* */
/** A convenient base class for creating your own NonlinearConstraint with 2
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, typename ERROR_FUNCTOR>
class NonlinearInequalityConstraint2: public NonlinearInequalityConstraint {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:

  typedef NonlinearInequalityConstraint2<VALUE1, VALUE2, ERROR_FUNCTOR> This;

private:
  static const int X1Dim = traits < VALUE1 > ::dimension;
  static const int X2Dim = traits < VALUE2 > ::dimension;

public:

  typedef NonlinearEqualityConstraint2<VALUE1, VALUE2, ERROR_FUNCTOR> ThisEquality;

  const ThisEquality equality_;

  /** methods to retrieve both keys */
  inline Key key1() const {
    return keys_[0];
  }
  inline Key key2() const {
    return keys_[1];
  }

  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequalityConstraint2(Key j1, Key j2, Key dualKey) :
      NonlinearInequalityConstraint(noiseModel::Constrained::All(1), cref_list_of<2>(j1)(j2),
          dualKey, false), equality_(j1, j2, dualKey) {
  }

  virtual ~NonlinearInequalityConstraint2() {
  }

  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector unwhitenedError(const Values &x,
      boost::optional<std::vector<Matrix> &> H = boost::none) const override {
    if (this->active(x)) {
      ERROR_FUNCTOR f;
      Vector signedError;
      if (H) {
        signedError = f(x.at < X1 > (keys_[0]), x.at < X2 > (keys_[1]), (*H)[0],
            (*H)[1]);
      } else {
        signedError = f(x.at < X1 > (keys_[0]), x.at < X2 > (keys_[1]));
      }
      for (int index = 0; index < signedError.size(); index++) {
        signedError[index] = signedError[index] > 0 ? signedError[index] : 0;
      }
      return signedError;
    } else {
      return Vector::Zero(this->dim());
    }
  }

  virtual GaussianFactor::shared_ptr multipliedHessian(const Values &x,
      const VectorValues &duals) const override {
    return equality_.multipliedHessian(x, duals);
  }

  virtual void evaluateHessians(const X1& x1, const X2& x2,
      std::vector<Matrix>& G11, std::vector<Matrix>& G12,
      std::vector<Matrix>& G22) const {
    equality_.evaluateHessians(x1, x2, G11, G12, G22);
  }
  
  virtual const NonlinearConstraint& getEquality() const {
    return equality_;
  }
  
private:
  /** Serialization function */
  friend class boost::serialization::access;

  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor",
            boost::serialization::base_object < Base > (*this));
  }
};
// \class NonlinearEqualityConstraint2

/**
 * A convinient base class for creating NonlinearInequalityConstraint with 3 variables. To
 * derive from this class implement computeError.
 */
template<class VALUE1, class VALUE2, class VALUE3, typename ERROR_FUNCTOR>
class NonlinearInequalityConstraint3: public NonlinearInequalityConstraint {

public:
  // typdefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;

protected:

  typedef NonlinearInequalityConstraint3<VALUE1, VALUE2, VALUE3, ERROR_FUNCTOR> This;

private:
  static const int X1Dim = traits < VALUE1 > ::dimension;
  static const int X2Dim = traits < VALUE2 > ::dimension;
  static const int X3Dim = traits < VALUE3 > ::dimension;

public:
  typedef NonlinearEqualityConstraint3<VALUE1, VALUE2, VALUE3, ERROR_FUNCTOR> ThisEquality;

  const ThisEquality equality_;

  inline Key key1() const {
    return keys_[0];
  }
  inline Key key2() const {
    return keys_[1];
  }
  inline Key key3() const {
    return keys_[2];
  }

  /**
   * Constructor.
   * ConstraintDim is the number of dimensions of the constraint error function == 1
   * @param j1
   * @param j3
   * @param dualKey
   * @return
   */
  NonlinearInequalityConstraint3(Key j1, Key j2, Key j3, Key dualKey) :
      NonlinearInequalityConstraint(noiseModel::Constrained::All(1), cref_list_of<3>(j1)(j2)(j3),
          dualKey, false), equality_(j1, j2, j3, dualKey) {
  }

  /** predefine evaluteError to return a 1-dimension vector */
  virtual Vector unwhitenedError(const Values &x,
      boost::optional<std::vector<Matrix> &> H = boost::none) const override {
    if (this->active(x)) {
      ERROR_FUNCTOR f;
      Vector signedError;
      if (H) {
        signedError = f(x.at < X1 > (keys_[0]), x.at < X2 > (keys_[1]), x.at < X3 > (keys_[2]), (*H)[0], (*H)[1], (*H)[2]);
      } else {
        signedError = f(x.at < X1 > (keys_[0]), x.at < X2 > (keys_[1]), x.at < X3 > (keys_[2]));
      }
      for (int index = 0; index < signedError.size(); index++) {
        signedError[index] = signedError[index] > 0 ? signedError[index] : 0;
      }
      return signedError;
    } else {
      return Vector::Zero(this->dim());
    }

  }
  virtual GaussianFactor::shared_ptr multipliedHessian(const Values &x,
      const VectorValues &duals) const override {
    return equality_.multipliedHessian(x, duals);
  }

  virtual void evaluateHessians(const X1& x1, const X2& x2, const X3& x3,
      std::vector<Matrix>& G11, std::vector<Matrix>& G12,
      std::vector<Matrix>& G13, std::vector<Matrix>& G22,
      std::vector<Matrix>& G23, std::vector<Matrix>& G33) const {
    equality_.evaluateHessians(x1, x2, x3, G11, G12, G13, G22, G23, G33);
  }
  
  virtual const NonlinearConstraint& getEquality() const {
    return equality_;
  }
private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor",
            boost::serialization::base_object < Base > (*this));
  }
};
} /* namespace gtsam */
