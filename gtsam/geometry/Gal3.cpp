/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Gal3.cpp
 * @brief   Implementation of 3D Galilean Group SGal(3) state
 * @authors Matt Kielo, Scott Baker, Frank Dellaert
 * @date    April 30, 2025
 *
 * This implementation is based on the paper:
 * Kelly, J. (2023). "All About the Galilean Group SGal(3)"
 * arXiv:2312.07555
 *
 * All section, equation, and page references in comments throughout this file
 * refer to the aforementioned paper.
 */


#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Event.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/geometry/concepts.h>

#include <iostream>
#include <cmath>
#include <functional>

namespace gtsam {

//------------------------------------------------------------------------------
// Constants and Helper function for Expmap/Logmap
//------------------------------------------------------------------------------
namespace { // Anonymous namespace for internal linkage
  constexpr double kSmallAngleThreshold = 1e-10;

  // Helper functions for accessing tangent vector components
  Eigen::Block<Vector10, 3, 1> rho(Vector10& v) { return v.block<3, 1>(0, 0); }
  Eigen::Block<Vector10, 3, 1> nu(Vector10& v) { return v.block<3, 1>(3, 0); }
  Eigen::Block<Vector10, 3, 1> theta(Vector10& v) { return v.block<3, 1>(6, 0); }
  Eigen::Block<Vector10, 1, 1> t_tan(Vector10& v) { return v.block<1, 1>(9, 0); }
  // Const versions
  Eigen::Block<const Vector10, 3, 1> rho(const Vector10& v) { return v.block<3, 1>(0, 0); }
  Eigen::Block<const Vector10, 3, 1> nu(const Vector10& v) { return v.block<3, 1>(3, 0); }
  Eigen::Block<const Vector10, 3, 1> theta(const Vector10& v) { return v.block<3, 1>(6, 0); }
  Eigen::Block<const Vector10, 1, 1> t_tan(const Vector10& v) { return v.block<1, 1>(9, 0); }

  // The 25x10 matrix of vectorized generators for SGal(3).
  // The columns correspond to perturbations in [rho, nu, theta, t_tan].
  // The tangent vector is [rho(3), nu(3), theta(3), t_tan(1)].
  // P_gal3 = [vec(G_rho), vec(G_nu), vec(G_theta), vec(G_t)]
  static const Eigen::Matrix<double, 25, 10> P_gal3 =
      (Eigen::Matrix<double, 25, 10>() <<
          // rho(0-2), nu(3-5), theta(6-8), t_tan(9)
          // --- M(i,0) --- rows 0-4
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          // --- M(i,1) --- rows 5-9
          0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          // --- M(i,2) --- rows 10-14
          0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          // --- M(i,3) --- rows 15-19
          0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          // --- M(i,4) --- rows 20-24
          1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0
          ).finished();

} // end anonymous namespace

//------------------------------------------------------------------------------
// Static Constructor/Create functions
//------------------------------------------------------------------------------
Gal3 Gal3::Create(const Rot3& R, const Point3& r, const Velocity3& v, double t,
                    OptionalJacobian<10, 3> H1, OptionalJacobian<10, 3> H2,
                    OptionalJacobian<10, 3> H3, OptionalJacobian<10, 1> H4) {
      if (H1) {
        H1->setZero();
        H1->block<3, 3>(6, 0) = Matrix3::Identity();
      }
      if (H2) {
        H2->setZero();
        H2->block<3, 3>(0, 0) = R.transpose();
      }
      if (H3) {
        H3->setZero();
        H3->block<3, 3>(3, 0) = R.transpose();
      }
      if (H4) {
        H4->setZero();
        Vector3 drho_dt = -R.transpose() * v;
        H4->block<3, 1>(0, 0) = drho_dt;
        (*H4)(9, 0) = 1.0;
      }
      return Gal3(R, r, v, t);
}

//------------------------------------------------------------------------------
Gal3 Gal3::FromPoseVelocityTime(const Pose3& pose, const Velocity3& v, double t,
                                OptionalJacobian<10, 6> H1, OptionalJacobian<10, 3> H2,
                                OptionalJacobian<10, 1> H3) {
    const Rot3& R = pose.rotation();
    const Point3& r = pose.translation();
    if (H1) {
        H1->setZero();
        H1->block<3, 3>(6, 0) = Matrix3::Identity();
        H1->block<3, 3>(0, 3) = Matrix3::Identity();
    }
    if (H2) {
        H2->setZero();
        H2->block<3, 3>(3, 0) = R.transpose();
    }
    if (H3) {
        H3->setZero();
        Vector3 drho_dt = -R.transpose() * v;
        H3->block<3, 1>(0, 0) = drho_dt;
        (*H3)(9, 0) = 1.0;
    }
    return Gal3(R, r, v, t);
}

//------------------------------------------------------------------------------
// Constructors
//------------------------------------------------------------------------------
Gal3::Gal3(const Matrix5& M) {
    // Constructor from 5x5 matrix representation (Equation 9, Page 5)
    if (std::abs(M(3, 3) - 1.0) > 1e-9 || std::abs(M(4, 4) - 1.0) > 1e-9 ||
        M.row(4).head(4).norm() > 1e-9 || M.row(3).head(3).norm() > 1e-9) {
        throw std::invalid_argument("Invalid Gal3 matrix structure: Check zero blocks and diagonal ones.");
    }
    R_ = Rot3(M.block<3, 3>(0, 0));
    v_ = M.block<3, 1>(0, 3);
    r_ = Point3(M.block<3, 1>(0, 4));
    t_ = M(3, 4);
}

//------------------------------------------------------------------------------
// Component Access
//------------------------------------------------------------------------------
const Rot3& Gal3::rotation(OptionalJacobian<3, 10> H) const {
    if (H) {
        H->setZero();
        H->block<3, 3>(0, 6) = Matrix3::Identity();
    }
    return R_;
}

//------------------------------------------------------------------------------
const Point3& Gal3::translation(OptionalJacobian<3, 10> H) const {
     if (H) {
        H->setZero();
        H->block<3,3>(0, 0) = R_.matrix();
        H->block<3,1>(0, 9) = v_;
    }
    return r_;
}

//------------------------------------------------------------------------------
const Velocity3& Gal3::velocity(OptionalJacobian<3, 10> H) const {
     if (H) {
        H->setZero();
        H->block<3, 3>(0, 3) = R_.matrix();
     }
    return v_;
}

//------------------------------------------------------------------------------
const double& Gal3::time(OptionalJacobian<1, 10> H) const {
    if (H) {
        H->setZero();
        (*H)(0, 9) = 1.0;
    }
    return t_;
}

//------------------------------------------------------------------------------
Matrix5 Gal3::matrix() const {
    // Returns 5x5 matrix representation as in Equation 9, Page 5
    Matrix5 M = Matrix5::Identity();
    M.block<3, 3>(0, 0) = R_.matrix();
    M.block<3, 1>(0, 3) = v_;
    M.block<3, 1>(0, 4) = Vector3(r_);
    M(3, 4) = t_;
    M.block<1,3>(3,0).setZero();
    M.block<1,4>(4,0).setZero();
    return M;
}

//------------------------------------------------------------------------------
Vector25 Gal3::vec(OptionalJacobian<25, 10> H) const {
    const Matrix5 T = this->matrix();
    if (H) {
        // The Jacobian is given by the formula H = (I_5 ⊗ T) * P_gal3
        // where P_gal3 is the matrix of vectorized generators.
        // This can be implemented efficiently with block-wise multiplication.
        *H << T * P_gal3.block<5, 10>(0, 0), T* P_gal3.block<5, 10>(5, 0),
            T* P_gal3.block<5, 10>(10, 0), T* P_gal3.block<5, 10>(15, 0),
            T* P_gal3.block<5, 10>(20, 0);
    }
    return Eigen::Map<const Vector25>(T.data());
}

//------------------------------------------------------------------------------
// Stream operator
//------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Gal3& state) {
    os << "R: " << state.R_ << "\n";
    os << "r: " << state.r_.transpose() << "\n";
    os << "v: " << state.v_.transpose() << "\n";
    os << "t: " << state.t_;
    return os;
}

//------------------------------------------------------------------------------
// Testable Requirements
//------------------------------------------------------------------------------
void Gal3::print(const std::string& s) const {
    std::cout << (s.empty() ? "" : s + " ");
    std::cout << *this << std::endl;
}

//------------------------------------------------------------------------------
bool Gal3::equals(const Gal3& other, double tol) const {
    return R_.equals(other.R_, tol) &&
           traits<Point3>::Equals(r_, other.r_, tol) &&
           traits<Velocity3>::Equals(v_, other.v_, tol) &&
           std::abs(t_ - other.t_) < tol;
}

//------------------------------------------------------------------------------
// Group Operations
//------------------------------------------------------------------------------
Gal3 Gal3::inverse() const {
    // Implements inverse formula from Equation 10, Page 5
    const Rot3 Rinv = R_.inverse();
    const Velocity3 v_inv = -(Rinv.rotate(v_));
    const Point3 r_inv = -(Rinv.rotate(Vector3(r_) - t_ * v_));
    const double t_inv = -t_;
    return Gal3(Rinv, r_inv, v_inv, t_inv);
}

//------------------------------------------------------------------------------
Gal3 Gal3::operator*(const Gal3& other) const {
    // Implements group composition through matrix multiplication
    const Gal3& g1 = *this;
    const Gal3& g2 = other;

    const Rot3 R_comp = g1.R_.compose(g2.R_);
    const Vector3 r1_vec(g1.r_);
    const Vector3 r2_vec(g2.r_);
    const Vector3 r_comp_vec = g1.R_.rotate(r2_vec) + g2.t_ * g1.v_ + r1_vec;
    const Velocity3 v_comp = g1.R_.rotate(g2.v_) + g1.v_;
    const double t_comp = g1.t_ + g2.t_;

    return Gal3(R_comp, Point3(r_comp_vec), v_comp, t_comp);
}

//------------------------------------------------------------------------------
// Lie Group Static Functions
//------------------------------------------------------------------------------
gtsam::Gal3 gtsam::Gal3::Expmap(const Vector10& xi, OptionalJacobian<10, 10> Hxi) {
    // Implements exponential map from Equations 16-19, Pages 7-8
    const Vector3 rho_tan = rho(xi);
    const Vector3 nu_tan = nu(xi);
    const Vector3 theta_tan = theta(xi);
    const double t_tan_val = t_tan(xi)(0);

    const gtsam::so3::DexpFunctor dexp_functor(theta_tan);
    const Rot3 R = Rot3::Expmap(theta_tan);
    const Matrix3 Jl_theta = dexp_functor.leftJacobian();

    Matrix3 E;
    if (dexp_functor.nearZero) {
         // Small angle approximation for E matrix (from Equation 19, Page 8)
         E = 0.5 * Matrix3::Identity() + (1.0 / 6.0) * dexp_functor.W + (1.0 / 24.0) * dexp_functor.WW;
    } else {
         // Closed form for E matrix (from Equation 19, Page 8)
         const double B_E = (1.0 - 2.0 * dexp_functor.B) / (2.0 * dexp_functor.theta2);
         E = 0.5 * Matrix3::Identity() + dexp_functor.C * dexp_functor.W + B_E * dexp_functor.WW;
    }

    const Point3 r_final = Point3(Jl_theta * rho_tan + E * (t_tan_val * nu_tan));
    const Velocity3 v_final = Jl_theta * nu_tan;
    const double t_final = t_tan_val;

    Gal3 result(R, r_final, v_final, t_final);

    if (Hxi) {
        *Hxi = Gal3::ExpmapDerivative(xi);
    }

    return result;
}

//------------------------------------------------------------------------------
Vector10 Gal3::Logmap(const Gal3& g, OptionalJacobian<10, 10> Hg) {
    // Implements logarithmic map from Equations 20-23, Page 8
    const Vector3 theta_vec = Rot3::Logmap(g.R_);
    const gtsam::so3::DexpFunctor dexp_functor_log(theta_vec);
    const Matrix3 Jl_theta_inv = dexp_functor_log.leftJacobianInverse();

    Matrix3 E;
    if (dexp_functor_log.nearZero) {
         // Small angle approximation for E matrix
         E = 0.5 * Matrix3::Identity() + (1.0 / 6.0) * dexp_functor_log.W + (1.0 / 24.0) * dexp_functor_log.WW;
    } else {
         // Closed form for E matrix (from Equation 19, Page 8)
         const double B_E = (1.0 - 2.0 * dexp_functor_log.B) / (2.0 * dexp_functor_log.theta2);
         E = 0.5 * Matrix3::Identity() + dexp_functor_log.C * dexp_functor_log.W + B_E * dexp_functor_log.WW;
    }

    const Vector3 r_vec = Vector3(g.r_);
    const Velocity3& v_vec = g.v_;
    const double& t_val = g.t_;

    // Implementation of Equation 23, Page 8
    const Vector3 nu_tan = Jl_theta_inv * v_vec;
    const Vector3 rho_tan = Jl_theta_inv * (r_vec - E * (t_val * nu_tan));
    const double t_tan_val = t_val;

    Vector10 xi;
    rho(xi) = rho_tan;
    nu(xi) = nu_tan;
    theta(xi) = theta_vec;
    t_tan(xi)(0) = t_tan_val;

    if (Hg) {
        *Hg = Gal3::LogmapDerivative(g);
    }

    return xi;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::AdjointMap() const {
    // Implements adjoint map as in Equation 26, Page 9
    const Matrix3 Rmat = R_.matrix();
    const Vector3 v_vec = v_;
    const Vector3 r_minus_tv = Vector3(r_) - t_ * v_;

    Matrix10 Ad = Matrix10::Zero();

    Ad.block<3,3>(0,0) = Rmat;
    Ad.block<3,3>(0,3) = -t_ * Rmat;
    Ad.block<3,3>(0,6) = skewSymmetric(r_minus_tv) * Rmat;
    Ad.block<3,1>(0,9) = v_vec;

    Ad.block<3,3>(3,3) = Rmat;
    Ad.block<3,3>(3,6) = skewSymmetric(v_vec) * Rmat;

    Ad.block<3,3>(6,6) = Rmat;

    Ad(9,9) = 1.0;

    return Ad;
}

//------------------------------------------------------------------------------
Vector10 Gal3::Adjoint(const Vector10& xi, OptionalJacobian<10, 10> H_g, OptionalJacobian<10, 10> H_xi) const {
    Matrix10 Ad = AdjointMap();
    Vector10 y = Ad * xi;

    if (H_xi) {
        *H_xi = Ad;
    }

    if (H_g) {
        // NOTE: Using numerical derivative for the Jacobian with respect to
        // the group element instead of deriving the analytical expression.
        // Future work to use analytical instead.
        std::function<Vector10(const Gal3&, const Vector10&)> adjoint_action_wrt_g =
          [&](const Gal3& g_in, const Vector10& xi_in) {
              return g_in.Adjoint(xi_in);
          };
        *H_g = numericalDerivative21(adjoint_action_wrt_g, *this, xi, 1e-7);
    }
    return y;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::adjointMap(const Vector10& xi) {
    // Implements adjoint representation as in Equation 28, Page 10
    const Matrix3 Theta_hat = skewSymmetric(theta(xi));
    const Matrix3 Nu_hat = skewSymmetric(nu(xi));
    const Matrix3 Rho_hat = skewSymmetric(rho(xi));
    const double t_val = t_tan(xi)(0);
    const Vector3 nu_vec = nu(xi);

    Matrix10 ad = Matrix10::Zero();

    ad.block<3,3>(0,0) = Theta_hat;
    ad.block<3,3>(0,3) = -t_val * Matrix3::Identity();
    ad.block<3,3>(0,6) = Rho_hat;
    ad.block<3,1>(0,9) = nu_vec;

    ad.block<3,3>(3,3) = Theta_hat;
    ad.block<3,3>(3,6) = Nu_hat;

    ad.block<3,3>(6,6) = Theta_hat;

    return ad;
}

//------------------------------------------------------------------------------
Vector10 Gal3::adjoint(const Vector10& xi, const Vector10& y, OptionalJacobian<10, 10> Hxi, OptionalJacobian<10, 10> Hy) {
    Matrix10 ad_xi = adjointMap(xi);
    if (Hy) *Hy = ad_xi;
    if (Hxi) {
         *Hxi = -adjointMap(y);
    }
    return ad_xi * y;
}

//------------------------------------------------------------------------------
static Matrix3 computeQMatrix(const so3::DexpFunctor& dexp, const Vector3& vec) {
    Matrix3 vec_hat = skewSymmetric(vec);
    Vector4 coeffs;
    if (dexp.nearZero) {
        coeffs << 0.5, 1.0 / 6.0, 0.0, 0.0;
    } else {
        coeffs << 0.5,
            dexp.C,
            (1.0 - 2.0 * dexp.B) / (2.0 * dexp.theta2),
            (2.0 * dexp.theta - 3.0 * std::sin(dexp.theta) + dexp.theta * std::cos(dexp.theta)) / (2.0 * dexp.theta2 * dexp.theta2 * dexp.theta);
    }
    
    Matrix3 T1 = vec_hat;
    Matrix3 T2 = dexp.W * vec_hat + vec_hat * dexp.W + dexp.W * vec_hat * dexp.W;
    Matrix3 T3 = dexp.WW * vec_hat + vec_hat * dexp.WW - 3.0 * (dexp.W * vec_hat * dexp.W);
    Matrix3 T4 = dexp.WW * vec_hat * dexp.W + dexp.W * vec_hat * dexp.WW;
    
    return coeffs(0) * T1 + coeffs(1) * T2 + coeffs(2) * T3 + coeffs(3) * T4;
}

static Matrix3 computeGal3LeftJacobianU1(const gtsam::so3::DexpFunctor& dexp) {
	// move into dexpfunctor?
  if (dexp.nearZero) {
    return 0.5 * Matrix3::Identity() + (1.0 / 3.0) * dexp.W;
  } else {
    return 0.5 * Matrix3::Identity() + (dexp.B - dexp.C) * dexp.W +
           ((0.5 - dexp.A + dexp.B) / dexp.theta2) * dexp.WW;
  }
}

static Matrix3 computeGamma1ThetaNuThetaTau(const so3::DexpFunctor& dexp, const Vector3& nu_vec) {
    Matrix3 nu_hat = skewSymmetric(nu_vec);
    Vector7 coeffs;
    if (dexp.nearZero) {
        coeffs << 1.0 / 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    } else {
        const double tn3 = dexp.theta * dexp.theta2;
        const double tn4 = dexp.theta2 * dexp.theta2;
        const double tn5 = dexp.theta * tn4;
        const double tn6 = dexp.theta2 * tn4;
        const double s = std::sin(dexp.theta);
        const double c = std::cos(dexp.theta);
        
        coeffs << 1.0 / 6.0,
            dexp.E,  
            (1.0 - 2.0 * dexp.B) / (2.0 * dexp.theta2), 
            (tn3 + 6.0 * dexp.theta + 6.0 * dexp.theta * c - 12.0 * s) / (6.0 * tn5),
            (12.0 * s - 12.0 * dexp.theta * c - 3.0 * dexp.theta2 * s - tn3) / (6.0 * tn5),
            (tn3 - 6.0 * dexp.theta + 6.0 * s) / (6.0 * tn5),
            (4.0 + dexp.theta2 + dexp.theta2 * c - 4.0 * dexp.theta * s - 4.0 * c) / (4.0 * tn6);
    }
    
    Matrix3 T0 = nu_hat;
    Matrix3 T1 = dexp.W * nu_hat;
    Matrix3 T2 = nu_hat * dexp.W;
    Matrix3 T3 = dexp.WW * nu_hat;
    Matrix3 T4 = dexp.W * nu_hat * dexp.W;
    Matrix3 T5 = nu_hat * dexp.WW;
    Matrix3 T6 = dexp.WW * nu_hat * dexp.W + dexp.W * nu_hat * dexp.WW;
    
    Matrix3 result = coeffs(0) * T0 + coeffs(1) * T1 + coeffs(2) * T2 + coeffs(3) * T3;
    result += coeffs(4) * T4 + coeffs(5) * T5 + coeffs(6) * T6;
    
    return result;
}

Matrix10 Gal3::ExpmapDerivative(const Vector10& xi) {
    Vector minus_xi = -xi;
    Vector3 rho_vec = minus_xi.segment<3>(0);
    Vector3 nu_vec = minus_xi.segment<3>(3);
    Vector3 theta_vec = minus_xi.segment<3>(6);
    double t_val = minus_xi(9);

    so3::DexpFunctor dexp(theta_vec);

    Matrix10 Jl_minus_xi = Matrix10::Zero();

    // Note: Because dexp is constructed on -theta, dexp.leftJacobian() returns Jr(theta)
    Matrix3 right_jacobian = dexp.leftJacobian();
    Jl_minus_xi.block<3,3>(0,0) = right_jacobian;   // ∂ρ/∂ρ block
    Jl_minus_xi.block<3,3>(3,3) = right_jacobian;   // ∂ν/∂ν block
    Jl_minus_xi.block<3,3>(6,6) = right_jacobian;   // ∂ϕ/∂ϕ block

    Jl_minus_xi.block<3,3>(0,3) = -computeGal3LeftJacobianU1(dexp) * t_val;
    Jl_minus_xi.block<3,3>(3,6) = computeQMatrix(dexp, nu_vec);
    Jl_minus_xi.block<3,3>(0,6) = computeQMatrix(dexp, rho_vec) - computeGamma1ThetaNuThetaTau(dexp, nu_vec)*t_val;
    
	Matrix3 gamma2;
    if (dexp.nearZero) {
        gamma2 = 0.5 * Matrix3::Identity() + 1.0 / 6.0 * dexp.W + (1.0/24.0) * dexp.WW;
    } else {
        gamma2 = 0.5 * Matrix3::Identity() + dexp.C * dexp.W + (1.0 - 2.0 * dexp.B) / (2.0 * dexp.theta2) * dexp.WW;
    }

    Jl_minus_xi.block<3,1>(0,9) = gamma2 * nu_vec;
   
    Jl_minus_xi(9,9) = 1.0;

    return Jl_minus_xi;
}

//------------------------------------------------------------------------------
Matrix10 Gal3::LogmapDerivative(const Gal3& g) {
    // Related to the inverse of left Jacobian in Equations 31-36, Pages 10-11
    // NOTE: Using numerical approximation instead of implementing the analytical
    // expression for the inverse Jacobian. Future work to replace this
    // with analytical derivative.
    Vector10 xi = Gal3::Logmap(g);
    if (xi.norm() < kSmallAngleThreshold) return Matrix10::Identity();
    std::function<Vector10(const Gal3&)> fn =
        [](const Gal3& g_in) { return Gal3::Logmap(g_in); };
    return numericalDerivative11<Vector10, Gal3>(fn, g, 1e-5);
}

//------------------------------------------------------------------------------
// Lie Algebra (Hat/Vee maps)
//------------------------------------------------------------------------------
Matrix5 Gal3::Hat(const Vector10& xi) {
    // Implements hat operator as in Equation 13, Page 6
    const Vector3 rho_tan = rho(xi);
    const Vector3 nu_tan = nu(xi);
    const Vector3 theta_tan = theta(xi);
    const double t_tan_val = t_tan(xi)(0);

    Matrix5 X = Matrix5::Zero();
    X.block<3, 3>(0, 0) = skewSymmetric(theta_tan);
    X.block<3, 1>(0, 3) = nu_tan;
    X.block<3, 1>(0, 4) = rho_tan;
    X(3, 4) = t_tan_val;
    return X;
}

//------------------------------------------------------------------------------
Vector10 Gal3::Vee(const Matrix5& X) {
    // Implements vee operator (inverse of hat operator in Equation 13, Page 6)
    if (X.row(4).norm() > 1e-9 || X.row(3).head(3).norm() > 1e-9 || std::abs(X(3,3)) > 1e-9) {
     throw std::invalid_argument("Matrix is not in sgal(3)");
    }

    Vector10 xi;
    rho(xi) = X.block<3, 1>(0, 4);
    nu(xi) = X.block<3, 1>(0, 3);
    const Matrix3& S = X.block<3, 3>(0, 0);
    theta(xi) << S(2, 1), S(0, 2), S(1, 0);
    t_tan(xi)(0) = X(3, 4);
    return xi;
}

//------------------------------------------------------------------------------
// ChartAtOrigin
//------------------------------------------------------------------------------
Gal3 Gal3::ChartAtOrigin::Retract(const Vector10& xi, ChartJacobian Hxi) {
  return Gal3::Expmap(xi, Hxi);
}

//------------------------------------------------------------------------------
Vector10 Gal3::ChartAtOrigin::Local(const Gal3& g, ChartJacobian Hg) {
  return Gal3::Logmap(g, Hg);
}

//------------------------------------------------------------------------------
Event Gal3::act(const Event& e, OptionalJacobian<4, 10> Hself,
                OptionalJacobian<4, 4> He) const {
  // Implements group action on events (spacetime points) as described in Section 4.1, Page 3-4
  const double& t_in = e.time();
  const Point3& p_in = e.location();

  const double t_out = t_in + t_;
  const Point3 p_out = R_.rotate(p_in) + v_ * t_in + r_;

  if (He) {
    He->setZero();
    (*He)(0, 0) = 1.0;
    He->block<3, 1>(1, 0) = v_;
    He->block<3, 3>(1, 1) = R_.matrix();
  }

  if (Hself) {
    Hself->setZero();
    const Matrix3 Rmat = R_.matrix();

    (*Hself)(0, 9) = 1.0;
    Hself->block<3, 3>(1, 0) = Rmat;
    Hself->block<3, 3>(1, 3) = Rmat * t_in;
    Hself->block<3, 3>(1, 6) = -Rmat * skewSymmetric(p_in);
    Hself->block<3, 1>(1, 9) = v_;
  }

  return Event(t_out, p_out);
}

} // namespace gtsam
