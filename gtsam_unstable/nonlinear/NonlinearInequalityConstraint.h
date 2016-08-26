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
/* ************************************************************************* */
/** A convenient base class for creating a nonlinear equality constraint with 1
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE, typename ERROR_FUNCTOR>
class NonlinearInequalityConstraint1: public NoiseModelFactor1<VALUE>,
    public NonlinearConstraint {

public:

  // typedefs for value types pulled from keys
  typedef VALUE X;

protected:

  typedef NoiseModelFactor1<VALUE> Base;
  typedef NonlinearInequalityConstraint1<VALUE, ERROR_FUNCTOR> This;

private:
  static const int X1Dim = traits < VALUE > ::dimension;
  
public:
  
  typedef NonlinearEqualityConstraint1<VALUE, ERROR_FUNCTOR> ThisEquality ;

  const ThisEquality equality_;
  
  /**
   * Constructor
   * @param j key of the variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequalityConstraint1(Key key, Key dualKey) :
      Base(noiseModel::Constrained::All(1), key),
      NonlinearConstraint(dualKey, false),
      equality_(key, dualKey) {
  }

  virtual ~NonlinearInequalityConstraint1() {
  }

  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H1 =
      boost::none) const {
    ERROR_FUNCTOR f;
    Vector signedError = f(x, H1);
    for(int index = 0; index < signedError.size(); index++){
      signedError[index] = signedError[index] > 0 ? signedError[index] : 0;
    }
    return signedError;
  }

  virtual GaussianFactor::shared_ptr multipliedHessian(const Values &x, const VectorValues &duals) const override {
    return equality_.multipliedHessian(x, duals);
  }
  
  virtual void evaluateHessians(const X& x1, std::vector<Matrix>& G11) const {
    equality_.evaluateHessians(x1, G11);
  }
  
private:
  
  /** Serialization function */
  friend class boost::serialization::access;
  
  template <class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version){
    ar
        & boost::serialization::make_nvp("NoiseModelFactor",
            boost::serialization::base_object < Base > (*this));
  }
};
// \class NonlinearEqualityConstraint1

/* ************************************************************************* */
/** A convenient base class for creating your own NonlinearConstraint with 2
 * variables.  To derive from this class, implement evaluateError(). */
template<class VALUE1, class VALUE2, typename ERROR_FUNCTOR>
class NonlinearInequalityConstraint2: public NoiseModelFactor2<
    VALUE1, VALUE2>, public NonlinearConstraint {

public:

  // typedefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;

protected:

  typedef NoiseModelFactor2<VALUE1, VALUE2> Base;
  typedef NonlinearInequalityConstraint2<VALUE1, VALUE2, ERROR_FUNCTOR> This;

private:
  static const int X1Dim = traits < VALUE1 > ::dimension;
  static const int X2Dim = traits < VALUE2 > ::dimension;

public:
  
  typedef NonlinearEqualityConstraint2<VALUE1, VALUE2, ERROR_FUNCTOR> ThisEquality;
  
  const ThisEquality equality_;
  
  /**
   * Constructor
   * @param j1 key of the first variable
   * @param j2 key of the second variable
   * @param constraintDim number of dimensions of the constraint error function
   */
  NonlinearInequalityConstraint2(Key j1, Key j2, Key dualKey) :
      Base(noiseModel::Constrained::All(1), j1, j2),
      NonlinearConstraint(dualKey, false),
      equality_(j1, j2, dualKey) {}

  virtual ~NonlinearInequalityConstraint2() {
  }

  /** predefine evaluateError to return a 1-dimension vector */
  virtual Vector evaluateError(const X1& x1, const X2& x2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    ERROR_FUNCTOR f;
    Vector signedError = f(x1, x2, H1, H2);
    for(int index = 0; index < signedError.size(); index++){
      signedError[index] = signedError[index] > 0 ? signedError[index] : 0;
    }
    return signedError;
  }

  virtual GaussianFactor::shared_ptr multipliedHessian(const Values &x, const VectorValues &duals) const override {
    return equality_.multipliedHessian(x, duals);
  }
  
  virtual void evaluateHessians(const X1& x1, const X2& x2, std::vector<Matrix>& G11,
                                std::vector<Matrix>& G12, std::vector<Matrix>& G22) const {
    equality_.evaluateHessians(x1, x2, G11, G12, G22);
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
class NonlinearInequalityConstraint3: public NoiseModelFactor3<
    VALUE1, VALUE2, VALUE3>, public NonlinearConstraint {

public:
  // typdefs for value types pulled from keys
  typedef VALUE1 X1;
  typedef VALUE2 X2;
  typedef VALUE3 X3;

protected:

  typedef NoiseModelFactor3<VALUE1, VALUE2, VALUE3> Base;
  typedef NonlinearInequalityConstraint3<VALUE1, VALUE2, VALUE3, ERROR_FUNCTOR> This;

private:
  static const int X1Dim = traits < VALUE1 > ::dimension;
  static const int X2Dim = traits < VALUE2 > ::dimension;
  static const int X3Dim = traits < VALUE3 > ::dimension;

public:
  typedef NonlinearEqualityConstraint3<VALUE1, VALUE2, VALUE3, ERROR_FUNCTOR> ThisEquality;

  const ThisEquality equality_;
  /**
   * Constructor.
   * ConstraintDim is the number of dimensions of the constraint error function == 1
   * @param j1
   * @param j3
   * @param dualKey
   * @return
   */
  NonlinearInequalityConstraint3(Key j1, Key j2, Key j3, Key dualKey) :
      Base(noiseModel::Constrained::All(1), j1, j2, j3),
      NonlinearConstraint(dualKey, false),
      equality_(j1, j2, j3, dualKey){}

  /** predefine evaluteError to return a 1-dimension vector */
  virtual Vector evaluateError(const X1 &x1, const X2 &x2, const X3 &x3,
      boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
      boost::optional<Matrix &> H3 = boost::none) const override {
    ERROR_FUNCTOR f;
    Vector signedError = f(x1, x2, x3, H1, H2, H3);
    for(int index = 0; index < signedError.size(); index++){
      signedError[index] = signedError[index] > 0 ? signedError[index] : 0;
    }
    return signedError;
  }

  virtual GaussianFactor::shared_ptr multipliedHessian(const Values &x, const VectorValues &duals) const override {
    return equality_.multipliedHessian(x, duals);
  }
  
  virtual void evaluateHessians(const X1& x1, const X2& x2, const X3& x3,
                                std::vector<Matrix>& G11, std::vector<Matrix>& G12,
                                std::vector<Matrix>& G13, std::vector<Matrix>& G22,
                                std::vector<Matrix>& G23, std::vector<Matrix>& G33) const{
    equality_.evaluateHessians(x1, x2, x3, G11, G12, G13, G22, G23, G33);
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
