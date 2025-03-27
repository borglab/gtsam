/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Similarity2.cpp
 * @brief  Implementation of Similarity2 transform
 * @author John Lambert, Varun Agrawal
 */

#include <gtsam/base/Manifold.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>

namespace gtsam {

using std::vector;

namespace internal {

/// Subtract centroids from point pairs.
static Point2Pairs SubtractCentroids(const Point2Pairs& abPointPairs,
                                     const Point2Pair& centroids) {
  Point2Pairs d_abPointPairs;
  for (const auto& [a, b] : abPointPairs) {
    Point2 da = a - centroids.first;
    Point2 db = b - centroids.second;
    d_abPointPairs.emplace_back(da, db);
  }
  return d_abPointPairs;
}

/// Form inner products x and y and calculate scale.
static double CalculateScale(const Point2Pairs& d_abPointPairs,
                             const Rot2& aRb) {
  double x = 0, y = 0;

  for (const auto& [da, db] : d_abPointPairs) {
    const Vector2 da_prime = aRb * db;
    y += da.transpose() * da_prime;
    x += da_prime.transpose() * da_prime;
  }
  const double s = y / x;
  return s;
}

/// Form outer product H.
static Matrix2 CalculateH(const Point2Pairs& d_abPointPairs) {
  Matrix2 H = Z_2x2;
  for (const auto& [da, db] : d_abPointPairs) {
    H += da * db.transpose();
  }
  return H;
}

/**
 * @brief This method estimates the similarity transform from differences point
 * pairs, given a known or estimated rotation and point centroids.
 *
 * @param d_abPointPairs
 * @param aRb
 * @param centroids
 * @return Similarity2
 */
static Similarity2 Align(const Point2Pairs& d_abPointPairs, const Rot2& aRb,
                         const Point2Pair& centroids) {
  const double s = CalculateScale(d_abPointPairs, aRb);
  // dividing aTb by s is required because the registration cost function
  // minimizes ||a - sRb - t||, whereas Sim(2) computes s(Rb + t)
  const Point2 aTb = (centroids.first - s * (aRb * centroids.second)) / s;
  return Similarity2(aRb, aTb, s);
}

/**
 * @brief This method estimates the similarity transform from point pairs,
 * given a known or estimated rotation.
 * Refer to:
 * http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
 * Chapter 3
 *
 * @param abPointPairs
 * @param aRb
 * @return Similarity2
 */
static Similarity2 AlignGivenR(const Point2Pairs& abPointPairs,
                               const Rot2& aRb) {
  auto centroids = means(abPointPairs);
  auto d_abPointPairs = internal::SubtractCentroids(abPointPairs, centroids);
  return internal::Align(d_abPointPairs, aRb, centroids);
}
}  // namespace internal

Similarity2::Similarity2() : t_(0, 0), s_(1) {}

Similarity2::Similarity2(double s) : t_(0, 0), s_(s) {}

Similarity2::Similarity2(const Rot2& R, const Point2& t, double s)
    : R_(R), t_(t), s_(s) {}

Similarity2::Similarity2(const Matrix2& R, const Vector2& t, double s)
    : R_(Rot2::ClosestTo(R)), t_(t), s_(s) {}

Similarity2::Similarity2(const Matrix3& T)
    : R_(Rot2::ClosestTo(T.topLeftCorner<2, 2>())),
      t_(T.topRightCorner<2, 1>()),
      s_(1.0 / T(2, 2)) {}

bool Similarity2::equals(const Similarity2& other, double tol) const {
  return R_.equals(other.R_, tol) &&
         traits<Point2>::Equals(t_, other.t_, tol) && s_ < (other.s_ + tol) &&
         s_ > (other.s_ - tol);
}

bool Similarity2::operator==(const Similarity2& other) const {
  return R_.matrix() == other.R_.matrix() && t_ == other.t_ && s_ == other.s_;
}

void Similarity2::print(const std::string& s) const {
  std::cout << std::endl;
  std::cout << s;
  rotation().print("\nR:\n");
  std::cout << "t: " << translation().transpose() << " s: " << scale()
            << std::endl;
}

Similarity2 Similarity2::Identity() { return Similarity2(); }

Similarity2 Similarity2::operator*(const Similarity2& S) const {
  return Similarity2(R_ * S.R_, ((1.0 / S.s_) * t_) + R_ * S.t_, s_ * S.s_);
}

Similarity2 Similarity2::inverse() const {
  const Rot2 Rt = R_.inverse();
  const Point2 sRt = Rt * (-s_ * t_);
  return Similarity2(Rt, sRt, 1.0 / s_);
}

Point2 Similarity2::transformFrom(const Point2& p) const {
  const Point2 q = R_ * p + t_;
  return s_ * q;
}

Pose2 Similarity2::transformFrom(const Pose2& T) const {
  Rot2 R = R_.compose(T.rotation());
  Point2 t = Point2(s_ * (R_ * T.translation() + t_));
  return Pose2(R, t);
}

Point2 Similarity2::operator*(const Point2& p) const {
  return transformFrom(p);
}

Similarity2 Similarity2::Align(const Point2Pairs& abPointPairs) {
  // Refer to Chapter 3 of
  // http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2005/Zinsser05-PSR.pdf
  if (abPointPairs.size() < 2)
    throw std::runtime_error("input should have at least 2 pairs of points");
  auto centroids = means(abPointPairs);
  auto d_abPointPairs = internal::SubtractCentroids(abPointPairs, centroids);
  Matrix2 H = internal::CalculateH(d_abPointPairs);
  // ClosestTo finds rotation matrix closest to H in Frobenius sense
  Rot2 aRb = Rot2::ClosestTo(H);
  return internal::Align(d_abPointPairs, aRb, centroids);
}

Similarity2 Similarity2::Align(const Pose2Pairs& abPosePairs) {
  const size_t n = abPosePairs.size();
  if (n < 2)
    throw std::runtime_error("input should have at least 2 pairs of poses");

  // calculate rotation
  vector<Rot2> rotations;
  Point2Pairs abPointPairs;
  rotations.reserve(n);
  abPointPairs.reserve(n);
  // Below denotes the pose of the i'th object/camera/etc
  // in frame "a" or frame "b".
  for (const auto& [aTi, bTi] : abPosePairs) {
    const Rot2 aRb = aTi.rotation().compose(bTi.rotation().inverse());
    rotations.emplace_back(aRb);
    abPointPairs.emplace_back(aTi.translation(), bTi.translation());
  }
  const Rot2 aRb_estimate = FindKarcherMean<Rot2>(rotations);

  return internal::AlignGivenR(abPointPairs, aRb_estimate);
}

Matrix2 Similarity2::GetV(double theta, double lambda) {
  // Derivation from https://ethaneade.com/lie_groups.pdf page 6
  const double lambda2 = lambda * lambda, theta2 = theta * theta;
  double A, B, C;
  if (theta2 > 1e-9) {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / theta2;
    C = (1 - A) / theta2;
  } else {
    // Taylor series expansion for theta=0
    A = 1.0;
    B = 0.5 - theta2 / 24.0;
    C = 1.0 / 6.0 - theta2 / 120.0;
  }
  double alpha = 1.0 / (1.0 + theta2 / lambda2);
  const double s = exp(lambda);

  double s_inv = 1.0 / s;
  double X = alpha * (1 - s_inv) / lambda + (1 - alpha) * (A - lambda * B);
  double Y =
      alpha * (s_inv - 1 + lambda) / lambda2 + (1 - alpha) * (B - lambda * C);

  Matrix2 V;
  V << X, -theta * Y, theta * Y, X;
  return V;
}

Vector4 Similarity2::Logmap(const Similarity2& S,  //
                            OptionalJacobian<4, 4> Hm) {
  const Vector1 w = Rot2::Logmap(S.R_);
  const double lambda = log(S.s_);
  // In Expmap, t = V * u -> in Logmap, u = V^{-1} * t
  Vector4 result;
  result << GetV(w[0], lambda).inverse() * S.t_, w, lambda;
  if (Hm) {
    throw std::runtime_error("Similarity2::Logmap: derivative not implemented");
  }
  return result;
}

Similarity2 Similarity2::Expmap(const Vector4& v,  //
                                OptionalJacobian<4, 4> Hm) {
  const Vector2 u = v.head<2>();
  const double theta = v[2];
  const double lambda = v[3];
  if (Hm) {
    throw std::runtime_error("Similarity2::Expmap: derivative not implemented");
  }
  const Matrix2 V = GetV(theta, lambda);
  return Similarity2(Rot2::Expmap(v.segment<1>(2)), V * u, exp(lambda));
}

Matrix4 Similarity2::AdjointMap() const {
  throw std::runtime_error("Similarity2::AdjointMap not implemented");
}

Matrix3 Similarity2::Hat(const Vector4 &xi) {
  const auto w = xi[2];
  const auto u = xi.head<2>();
  const double lambda = xi[3];
  Matrix3 W;
  W << 0, -w, u[0],
       w,  0, u[1],
       0,  0, -lambda;
  return W;
}

Vector4 Similarity2::Vee(const Matrix3 &Xi) {
  Vector4 xi;
  xi[2] = Xi(1, 0);
  xi.head<2>() = Xi.topRightCorner<2, 1>();
  xi[3] = -Xi(2, 2);
  return xi;
}
std::ostream& operator<<(std::ostream& os, const Similarity2& p) {
  os << "[" << p.rotation().theta() << " " << p.translation().transpose() << " "
     << p.scale() << "]\';";
  return os;
}

Matrix3 Similarity2::matrix() const {
  Matrix3 T;
  T.topRows<2>() << R_.matrix(), t_;
  T.bottomRows<1>() << 0, 0, 1.0 / s_;
  return T;
}

}  // namespace gtsam
