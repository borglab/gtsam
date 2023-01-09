/**
 * @file testPendulumExplicitEuler.cpp
 * @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/dynamics/SimpleHelicopter.h>
#include "gtsam/base/Vector.h"
#include "gtsam/geometry/Pose3.h"

/* ************************************************************************* */
using namespace std::placeholders;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;

const double tol=1e-5;
const double h = 0.01;

//const double deg2rad = M_PI/180.0;
//Pose3 g1(Rot3::Ypr(deg2rad*10.0, deg2rad*20.0, deg2rad*30.0), Point3(100.0, 200.0, 300.0));
Pose3 g1(Rot3(), Point3(100.0, 0.0, 300.0));
//Vector6 v1((Vector(6) << 0.1, 0.05, 0.02, 10.0, 20.0, 30.0).finished());
Vector6 V1_w((Vector(6) << 0.0, 0.0, M_PI/3, 0.0, 0.0, 30.0).finished());
Vector6 V1_g1 = g1.inverse().Adjoint(V1_w);
Pose3 g2(g1.expmap(h*V1_g1));
//Vector6 v2 = Pose3::Logmap(g1.between(g2));

double mass = 100.0;
Vector gamma2 = Vector2(0.0, 0.0);  // no shape
Vector u2 = Vector2(0.0, 0.0); // no control at time 2
double distT = 1.0; // distance from the body-centered x axis to the big top motor
double distR = 5.0; // distance from the body-centered z axis to the small motor
Matrix Mass = ((Vector(3) << mass, mass, mass).finished()).asDiagonal();
Matrix Inertia = (Vector(6) << 2.0/5.0*mass*distR*distR, 2.0/5.0*mass*distR*distR, 2.0/5.0*mass*distR*distR, mass, mass, mass).finished().asDiagonal();

Vector computeFu(const Vector& gamma, const Vector& control) {
  double gamma_r = gamma(0), gamma_p = gamma(1);

  Matrix F = (Matrix(6, 2) << distT*sin(gamma_r), 0.0,
                           distT*sin(gamma_p*cos(gamma_r)), 0.0,
                           0.0, distR,
                           sin(gamma_p)*cos(gamma_r), 0.0,
                          -sin(gamma_r), -1.0,
                           cos(gamma_p)*sin(gamma_r), 0.0
                            ).finished();
  return F*control;
}

/* ************************************************************************* */
TEST( Reconstruction, evaluateError) {
  // hard constraints don't need a noise model
  Reconstruction constraint(G(2), G(1), V(1), h);

  // verify error function
  Matrix H1, H2, H3;
  EXPECT(
      assert_equal(Z_6x1, constraint.evaluateError(g2, g1, V1_g1, H1, H2, H3), tol));

  std::function<Vector(const Pose3&, const Pose3&, const Vector6&)> f = [&constraint](const Pose3& a1, const Pose3& a2,
                                                                                      const Vector6& a3) {
    return constraint.evaluateError(a1, a2, a3);
  };
  Matrix numericalH1 = numericalDerivative31(f, g2, g1, V1_g1, 1e-5);

  Matrix numericalH2 = numericalDerivative32(f, g2, g1, V1_g1, 1e-5);

  Matrix numericalH3 = numericalDerivative33(f, g2, g1, V1_g1, 1e-5);

  EXPECT(assert_equal(numericalH1,H1,1e-5));
  EXPECT(assert_equal(numericalH2,H2,1e-5));
#ifdef GTSAM_USE_QUATERNIONS // TODO: why is the quaternion version much less accurate??
  EXPECT(assert_equal(numericalH3,H3,1e-3));
#else
  EXPECT(assert_equal(numericalH3,H3,1e-3));
#endif
}

/* ************************************************************************* */
// Implement Newton-Euler equation for rigid body dynamics
Vector newtonEuler(const Vector& Vb, const Vector& Fb, const Matrix& Inertia) {
  Matrix W = Pose3::adjointMap((Vector(6) << Vb(0), Vb(1), Vb(2), 0., 0., 0.).finished());
  Vector dV = Inertia.inverse()*(Fb - W*Inertia*Vb);
  return dV;
}

TEST( DiscreteEulerPoincareHelicopter, evaluateError) {
  Vector Fu = computeFu(gamma2, u2);
  Vector fGravity_g1 = Z_6x1;
  fGravity_g1.segment<3>(3) = g1.rotation().unrotate(Vector3(0, 0, -mass*9.81));  // gravity force in g1 frame
  Vector Fb = Fu+fGravity_g1;

  Vector dV = newtonEuler(V1_g1, Fb, Inertia);
  Vector V2_g1 = dV*h + V1_g1;
  Pose3 g21 = g2.between(g1);
  Vector V2_g2 = g21.Adjoint(V2_g1);  // convert the new velocity to g2's frame

  Vector6 expectedv2(V2_g2);

  // hard constraints don't need a noise model
  DiscreteEulerPoincareHelicopter constraint(V(2), V(1), G(2), h,
      Inertia, Fu, mass);

  // verify error function
  Matrix H1, H2, H3;
  EXPECT(assert_equal(Z_6x1, constraint.evaluateError(expectedv2, V1_g1, g2, H1, H2, H3), 1e0));

  std::function<Vector(const Vector6&, const Vector6&, const Pose3&)> f =
      [&constraint](const Vector6& a1, const Vector6& a2, const Pose3& a3) {
        return constraint.evaluateError(a1, a2, a3);
      };

  Matrix numericalH1 = numericalDerivative31(f, expectedv2, V1_g1, g2, 1e-5);

  Matrix numericalH2 = numericalDerivative32(f, expectedv2, V1_g1, g2, 1e-5);

  Matrix numericalH3 = numericalDerivative33(f, expectedv2, V1_g1, g2, 1e-5);

  EXPECT(assert_equal(numericalH1,H1,1e-5));
  EXPECT(assert_equal(numericalH2,H2,1e-5));
  EXPECT(assert_equal(numericalH3,H3,5e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
