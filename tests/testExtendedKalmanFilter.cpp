/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExtendedKalmanFilter
 * @author Stephen Williams
 */

// TODO: Perform tests with nontrivial data
// TODO: Perform tests with nonlinear data

#include <CppUnitLite/TestHarness.h>

#include <gtsam/nonlinear/ExtendedKalmanFilter-inl.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/LieValues-inl.h>
#include <gtsam/geometry/Point2.h>

using namespace gtsam;

// Define Types for Linear System Test
typedef TypedSymbol<Point2, 'x'> LinearKey;
typedef LieValues<LinearKey> LinearValues;
typedef Point2 LinearMeasurement;

/* ************************************************************************* */
TEST( ExtendedKalmanFilter, linear ) {

  // Create the Kalman Filter initialization point
  Point2 x_initial(0.0, 0.0);
  SharedDiagonal P_initial = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1));

  // Create an ExtendedKalmanFilter object
  ExtendedKalmanFilter<LinearValues,LinearKey> ekf(x_initial, P_initial);

  // Create the keys for our example
  LinearKey x0(0), x1(1), x2(2), x3(3);

  // Create the controls and measurement properties for our example
  double dt = 1.0;
  Vector u = Vector_(2, 1.0, 0.0);
  Point2 difference(u*dt);
  SharedDiagonal Q = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1), true);
  Point2 z1(1.0, 0.0);
  Point2 z2(2.0, 0.0);
  Point2 z3(3.0, 0.0);
  SharedDiagonal R = noiseModel::Diagonal::Sigmas(Vector_(2, 0.25, 0.25), true);

  // Create the set of expected output values
  Point2 expected1(1.0, 0.0);
  Point2 expected2(2.0, 0.0);
  Point2 expected3(3.0, 0.0);

  // Run iteration 1
  // Create motion factor
  BetweenFactor<LinearValues,LinearKey> factor1(x0, x1, difference, Q);
  Point2 predict1 = ekf.predict(factor1);
  EXPECT(assert_equal(expected1,predict1));

  // Create the measurement factor
  PriorFactor<LinearValues,LinearKey> factor2(x1, z1, R);
  Point2 update1 = ekf.update(factor2);
  EXPECT(assert_equal(expected1,update1));

  // Run iteration 2
  BetweenFactor<LinearValues,LinearKey> factor3(x1, x2, difference, Q);
  Point2 predict2 = ekf.predict(factor3);
  EXPECT(assert_equal(expected2,predict2));

  PriorFactor<LinearValues,LinearKey> factor4(x2, z2, R);
  Point2 update2 = ekf.update(factor4);
  EXPECT(assert_equal(expected2,update2));

  // Run iteration 3
  BetweenFactor<LinearValues,LinearKey> factor5(x2, x3, difference, Q);
  Point2 predict3 = ekf.predict(factor5);
  EXPECT(assert_equal(expected3,predict3));

  PriorFactor<LinearValues,LinearKey> factor6(x3, z3, R);
  Point2 update3 = ekf.update(factor6);
  EXPECT(assert_equal(expected3,update3));

  return;
}


// Create Motion Model Factor
class NonlinearMotionModel : public NonlinearFactor2<LinearValues,LinearKey,LinearKey> {
public:
  typedef typename LinearKey::Value T;

private:
  typedef NonlinearFactor2<LinearValues,LinearKey,LinearKey> Base;
  typedef NonlinearMotionModel This;

protected:
  Matrix Q_;
  Matrix Q_invsqrt_;

public:
  NonlinearMotionModel(){}

  NonlinearMotionModel(const LinearKey& key1, const LinearKey& key2) :
    Base(noiseModel::Diagonal::Sigmas(Vector_(2, 1.0, 1.0)), key1, key2), Q_(2,2) {

    // Initialize motion model parameters:
    // w is vector of motion noise sigmas. The final covariance is calculated as G*w*w'*G'
    // TODO: Ultimately, the value Q^-1/2 is needed. This can be calculated/derived symbolically, eliminating the
    // need for the noiseModel_, and improving computational performance.
    Matrix G(2,2);
    Matrix w(2,2);

    G << 1.0, 0.0, 0.0, 1.0;
    w << 1.0, 0.0, 0.0, 1.0;

    w = G*w;
    Q_ = w*w.transpose();
    Q_invsqrt_ = inverse_square_root(Q_);
  }

  virtual ~NonlinearMotionModel() {}

  // Calculate the next state prediction using the nonlinear function f()
  T f(const T& x_t0) const {

    // Calculate f(x)
    double x = x_t0.x() * x_t0.x();
    double y = x_t0.x() * x_t0.y();
    T x_t1(x,y);

    // In this example, the noiseModel remains constant
    return x_t1;
  }

  // Calculate the Jacobian of the nonlinear function f()
  Matrix F(const T& x_t0) const {
    // Calculate Jacobian of f()
    Matrix F = Matrix(2,2);
    F(0,0) = 2*x_t0.x();
    F(0,1) = 0.0;
    F(1,0) = x_t0.y();
    F(1,1) = x_t0.x();

    return F;
  }

  // Calculate the inverse square root of the motion model covariance, Q
  Matrix QInvSqrt(const T& x_t0) const {
    // This motion model has a constant covariance
    return Q_invsqrt_;
  }

  /* print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearMotionModel\n";
    std::cout << "  key1: " << (std::string) key1_ << std::endl;
    std::cout << "  key2: " << (std::string) key2_ << std::endl;
  }

  /** Check if two factors are equal. Note type is IndexFactor and needs cast. */
  virtual bool equals(const NonlinearFactor2<LinearValues,LinearKey,LinearKey>& f, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*> (&f);
    return (e != NULL) && (key1_ == e->key1_) && (key2_ == e->key2_);
  }

  /**
   * calculate the error of the factor
   * Override for systems with unusual noise models
   */
  virtual double error(const LinearValues& c) const {
    Vector w = whitenedError(c);
    return 0.5 * w.dot(w);
  }

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const {
    return 2;
  }

  /** Vector of errors, whitened ! */
  Vector whitenedError(const LinearValues& c) const {
    return QInvSqrt(c[key1_])*unwhitenedError(c);
  }

  /**
   * Linearize a non-linearFactor2 to get a GaussianFactor
   * Ax-b \approx h(x1+dx1,x2+dx2)-z = h(x1,x2) + A2*dx1 + A2*dx2 - z
   * Hence b = z - h(x1,x2) = - error_vector(x)
   */
  boost::shared_ptr<GaussianFactor> linearize(const LinearValues& c, const Ordering& ordering) const {
    const X1& x1 = c[key1_];
    const X2& x2 = c[key2_];
    Matrix A1, A2;
    Vector b = -evaluateError(x1, x2, A1, A2);
    const Index var1 = ordering[key1_], var2 = ordering[key2_];
    SharedDiagonal constrained =
        boost::shared_dynamic_cast<noiseModel::Constrained>(this->noiseModel_);
    if (constrained.get() != NULL) {
      return JacobianFactor::shared_ptr(new JacobianFactor(var1, A1, var2,
          A2, b, constrained));
    }
    // "Whiten" the system before converting to a Gaussian Factor
    Matrix Qinvsqrt = QInvSqrt(x1);
    A1 = Qinvsqrt*A1;
    A2 = Qinvsqrt*A2;
    b = Qinvsqrt*b;
    return GaussianFactor::shared_ptr(new JacobianFactor(var1, A1, var2,
        A2, b, noiseModel::Unit::Create(b.size())));
  }

  /** vector of errors */
  Vector evaluateError(const T& p1, const T& p2,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {

    // error = p2 - f(p1)
    // H1 = d error / d p1 = -d f/ d p1 = -F
    // H2 = d error / d p2 = I
    T prediction = f(p1);

    if(H1){
      *H1 = -F(p1);
    }

    if(H2){
      *H2 = eye(dim());
    }

    // Return the error between the prediction and the supplied value of p2
    return prediction.logmap(p2);
  }

};

// Create Measurement Model Factor
class NonlinearMeasurementModel : public NonlinearFactor1<LinearValues,LinearKey> {
public:
  typedef typename LinearKey::Value T;

private:

  typedef NonlinearFactor1<LinearValues,LinearKey> Base;
  typedef NonlinearMeasurementModel This;

protected:
  Vector z_; /** The measurement */
  Matrix R_; /** The measurement error covariance */
  Matrix R_invsqrt_; /** The inv sqrt of the measurement error covariance */

public:
  NonlinearMeasurementModel(){}

  NonlinearMeasurementModel(const LinearKey& key, Vector z) :
    Base(noiseModel::Diagonal::Sigmas(Vector_(2, 1.0, 1.0)), key), z_(z), R_(1,1) {

    // Initialize nonlinear measurement model parameters:
    // z(t) = H*x(t) + v
    // where v ~ N(0, noiseModel_)
    R_ << 1.0;
    R_invsqrt_ = inverse_square_root(R_);
  }

  virtual ~NonlinearMeasurementModel() {}

  // Calculate the predicted measurement using the nonlinear function h()
  // Byproduct: updates Jacobian H, and noiseModel (R)
  Vector h(const T& x_t1) const {

    // Calculate prediction
    Vector z_hat(1);
    z_hat(0) = 2*x_t1.x()*x_t1.x() - x_t1.x()*x_t1.y() - 2.5*x_t1.x() + 7*x_t1.y();

    return z_hat;
  }

  Matrix H(const T& x_t1) const {
    // Update Jacobian
    Matrix H(1,2);
    H(0,0) =  4*x_t1.x() - x_t1.y() - 2.5;
    H(0,1) = -1*x_t1.x() + 7;

    return H;
  }

  // Calculate the inverse square root of the motion model covariance, Q
  Matrix RInvSqrt(const T& x_t0) const {
    // This motion model has a constant covariance
    return R_invsqrt_;
  }

  /* print */
  virtual void print(const std::string& s = "") const {
    std::cout << s << ": NonlinearMeasurementModel\n";
    std::cout << "  key: " << (std::string) key_ << std::endl;
  }

  /** Check if two factors are equal. Note type is IndexFactor and needs cast. */
  virtual bool equals(const NonlinearFactor1<LinearValues,LinearKey>& f, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*> (&f);
    return (e != NULL) && (key_ == e->key_);
  }

  /**
   * calculate the error of the factor
   * Override for systems with unusual noise models
   */
  virtual double error(const LinearValues& c) const {
    Vector w = whitenedError(c);
    return 0.5 * w.dot(w);
  }

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const {
    return 1;
  }

  /** Vector of errors, whitened ! */
  Vector whitenedError(const LinearValues& c) const {
    return RInvSqrt(c[key_])*unwhitenedError(c);
  }

  /**
   * Linearize a nonlinearFactor1 measurement factor into a GaussianFactor
   * Ax-b \approx h(x1+dx1)-z = h(x1) + A1*dx1 - z
   * Hence b = z - h(x1) = - error_vector(x)
   */
  boost::shared_ptr<GaussianFactor> linearize(const LinearValues& c, const Ordering& ordering) const {
    const X& x1 = c[key_];
    Matrix A1;
    Vector b = -evaluateError(x1, A1);
    const Index var1 = ordering[key_];
    SharedDiagonal constrained =
        boost::shared_dynamic_cast<noiseModel::Constrained>(this->noiseModel_);
    if (constrained.get() != NULL) {
      return JacobianFactor::shared_ptr(new JacobianFactor(var1, A1, b, constrained));
    }
    // "Whiten" the system before converting to a Gaussian Factor
    Matrix Rinvsqrt = RInvSqrt(x1);
    A1 = Rinvsqrt*A1;
    b = Rinvsqrt*b;
    return GaussianFactor::shared_ptr(new JacobianFactor(var1, A1, b, noiseModel::Unit::Create(b.size())));
  }

  /** vector of errors */
  Vector evaluateError(const LinearKey::Value& p, boost::optional<Matrix&> H1 = boost::none) const {
    // error = z - h(p)
    // H = d error / d p = -d h/ d p = -H
    Vector z_hat = h(p);

    if(H1){
      *H1 = -H(p);
    }

    // Return the error between the prediction and the supplied value of p2
    return z_ - z_hat;
  }

};


/* ************************************************************************* */
TEST( ExtendedKalmanFilter, nonlinear ) {

  // Create the set of expected output values (generated using Matlab Kalman Filter)
  Point2 expected_predict[10];
  Point2 expected_update[10];
  expected_predict[1] = Point2(0.81,0.99);
  expected_update[1] =  Point2(0.824926197027,0.29509808);
  expected_predict[2] = Point2(0.680503230541,0.24343413);
  expected_update[2] =  Point2(0.773360464065,0.43518355);
  expected_predict[3] = Point2(0.598086407378,0.33655375);
  expected_update[3] =  Point2(0.908781566884,0.60766713);
  expected_predict[4] = Point2(0.825883936308,0.55223668);
  expected_update[4] =  Point2(1.23221370495,0.74372849);
  expected_predict[5] = Point2(1.51835061468,0.91643243);
  expected_update[5] =  Point2(1.32823362774,0.855855);
  expected_predict[6] = Point2(1.76420456986,1.1367754);
  expected_update[6] =  Point2(1.36378040243,1.0623832);
  expected_predict[7] = Point2(1.85989698605,1.4488574);
  expected_update[7] =  Point2(1.24068063304,1.3431964);
  expected_predict[8] = Point2(1.53928843321,1.6664778);
  expected_update[8] =  Point2(1.04229257957,1.4856408);
  expected_predict[9] = Point2(1.08637382142,1.5484724);
  expected_update[9] =  Point2(1.13201933157,1.5795507);
  expected_predict[10] = Point2(1.28146776705,1.7880819);
  expected_update[10] =  Point2(1.22159588179,1.7434982);

  // Create the Kalman Filter initialization point
  Point2 x_initial(0.90, 1.10);
  SharedDiagonal P_initial = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1));

  // Create an ExtendedKalmanFilter object
  ExtendedKalmanFilter<LinearValues,LinearKey> ekf(x_initial, P_initial);

  // Enter Predict-Update Loop
  Point2 x_predict, x_update;
  for(unsigned int i = 1; i < 9; ++i){
    // Create motion factor
    NonlinearMotionModel motionFactor(LinearKey(i-1), LinearKey(i));
    x_predict = ekf.predict(motionFactor);

    // Create a measurement factor
    NonlinearMeasurementModel measurementFactor(LinearKey(i), Vector_(1, (double)i));
    x_update = ekf.update(measurementFactor);

    EXPECT(assert_equal(expected_predict[i],x_predict, 1e-6));
    EXPECT(assert_equal(expected_update[i], x_update,  1e-6));
  }

  return;
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
