/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ExtendedPose3-inl.h
 * @brief   Template implementations for ExtendedPose3<K, Derived>
 * @author  Frank Dellaert, et al.
 */

#pragma once

namespace gtsam {

template <int K, class Derived>
size_t ExtendedPose3<K, Derived>::RuntimeK(const TangentVector& xi) {
  if constexpr (K == Eigen::Dynamic) {
    assert(xi.size() >= 3 && (xi.size() - 3) % 3 == 0);
    return static_cast<size_t>((xi.size() - 3) / 3);
  } else {
    return static_cast<size_t>(K);
  }
}

template <int K, class Derived>
void ExtendedPose3<K, Derived>::ZeroJacobian(ChartJacobian H, Eigen::Index d) {
  if (!H) return;
  if constexpr (dimension == Eigen::Dynamic) {
    H->setZero(d, d);
  } else {
    (void)d;
    H->setZero();
  }
}

template <int K, class Derived>
ExtendedPose3<K, Derived>::ExtendedPose3(const Rot3& R, const Matrix3K& x)
    : R_(R), t_(x) {}

template <int K, class Derived>
template <int FixedK, typename, typename... Vecs, typename>
ExtendedPose3<K, Derived>::ExtendedPose3(const Rot3& R, const Vecs&... xs)
    : R_(R), t_(Matrix3K::Zero()) {
  const Point3 columns[] = {Point3(xs)...};
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(FixedK); ++i) {
    t_.col(i) = columns[i];
  }
}

template <int K, class Derived>
ExtendedPose3<K, Derived>::ExtendedPose3(const MatrixRep& T) {
  const Eigen::Index n = T.rows();
  if constexpr (K == Eigen::Dynamic) {
    if (T.cols() != n || n < 3) {
      throw std::invalid_argument("ExtendedPose3: invalid matrix shape.");
    }
    t_.resize(3, n - 3);
  } else {
    if (n != matrixDim || T.cols() != matrixDim) {
      throw std::invalid_argument("ExtendedPose3: invalid matrix shape.");
    }
  }

  R_ = Rot3(T.template block<3, 3>(0, 0));
  t_ = T.block(0, 3, 3, n - 3);
}

template <int K, class Derived>
const Rot3& ExtendedPose3<K, Derived>::rotation(ComponentJacobian H) const {
  if (H) {
    if constexpr (dimension == Eigen::Dynamic) {
      H->setZero(3, static_cast<Eigen::Index>(dim()));
    } else {
      H->setZero();
    }
    H->block(0, 0, 3, 3) = I_3x3;
  }
  return R_;
}

template <int K, class Derived>
Point3 ExtendedPose3<K, Derived>::x(size_t i, ComponentJacobian H) const {
  if (i >= k()) throw std::out_of_range("ExtendedPose3: x(i) out of range.");
  if (H) {
    if constexpr (dimension == Eigen::Dynamic) {
      H->setZero(3, static_cast<Eigen::Index>(dim()));
    } else {
      H->setZero();
    }
    const Eigen::Index idx = 3 + 3 * static_cast<Eigen::Index>(i);
    H->block(0, idx, 3, 3) = R_.matrix();
  }
  return t_.col(static_cast<Eigen::Index>(i));
}

template <int K, class Derived>
const typename ExtendedPose3<K, Derived>::Matrix3K&
ExtendedPose3<K, Derived>::xMatrix() const {
  return t_;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::Matrix3K&
ExtendedPose3<K, Derived>::xMatrix() {
  return t_;
}

template <int K, class Derived>
void ExtendedPose3<K, Derived>::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

template <int K, class Derived>
bool ExtendedPose3<K, Derived>::equals(const ExtendedPose3& other,
                                       double tol) const {
  return R_.equals(other.R_, tol) && equal_with_abs_tol(t_, other.t_, tol);
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::This ExtendedPose3<K, Derived>::inverse()
    const {
  const Rot3 Rt = R_.inverse();
  const Matrix3K x = -(Rt.matrix() * t_);
  return MakeReturn(ExtendedPose3(Rt, x));
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::This ExtendedPose3<K, Derived>::operator*(
    const This& other) const {
  const ExtendedPose3& otherBase = AsBase(other);
  if constexpr (K == Eigen::Dynamic) {
    if (k() != otherBase.k()) {
      throw std::invalid_argument(
          "ExtendedPose3: compose requires matching k.");
    }
  }
  Matrix3K x = t_ + R_.matrix() * otherBase.t_;
  return MakeReturn(ExtendedPose3(R_ * otherBase.R_, x));
}

// Expmap is implemented in so3::ExpmapFunctor::expmap, based on Ethan Eade's
// elegant Lie group document, at https://www.ethaneade.org/lie.pdf.
// See also [this document](doc/Jacobians.md)
template <int K, class Derived>
typename ExtendedPose3<K, Derived>::This ExtendedPose3<K, Derived>::Expmap(
    const TangentVector& xi, ChartJacobian Hxi) {
  // Get angular velocity omega
  const Vector3 w = xi.template head<3>();

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  // Compute rotation using Expmap
#ifdef GTSAM_USE_QUATERNIONS
  // Reuse any quaternion-specific validation inside Rot3::Expmap.
  const Rot3 R = Rot3::Expmap(w);
#else
  const Rot3 R(local.expmap());
#endif

  const Eigen::Index k = static_cast<Eigen::Index>(RuntimeK(xi));

  // The translation t = local.Jacobian().left() * v.
  // Below we call local.Jacobian().applyLeft, which is faster if you don't need
  // Jacobians, and returns Jacobian of t with respect to w if asked.
  // NOTE(Frank): this does the same as the intuitive formulas:
  //   t_parallel = w * w.dot(v);  // translation parallel to axis
  //   w_cross_v = w.cross(v);     // translation orthogonal to axis
  //   t = (w_cross_v - Rot3::Expmap(w) * w_cross_v + t_parallel) / theta2;
  // but Local does not need R, deals automatically with the case where theta2
  // is near zero, and also gives us the machinery for the Jacobians.

  Matrix3K x;
  if constexpr (K == Eigen::Dynamic) x.resize(3, k);

  if (Hxi) {
    ZeroJacobian(Hxi, 3 + 3 * k);
    const Matrix3 Jr = local.Jacobian().right();
    Hxi->block(0, 0, 3, 3) = Jr;  // Jr here *is* the Jacobian of expmap
    const Matrix3 Rt = R.transpose();
    for (Eigen::Index i = 0; i < k; ++i) {
      Matrix3 H_xi_w;
      const Eigen::Index idx = 3 + 3 * i;
      const Vector3 rho = xi.template segment<3>(idx);
      x.col(i) = local.Jacobian().applyLeft(rho, &H_xi_w);
      Hxi->block(idx, 0, 3, 3) = Rt * H_xi_w;
      Hxi->block(idx, idx, 3, 3) = Jr;
      // In the last row, Jr = R^T * Jl, see Barfoot eq. (8.83).
      // Jl is the left Jacobian of SO(3) at w.
    }
  } else {
    for (Eigen::Index i = 0; i < k; ++i) {
      const Eigen::Index idx = 3 + 3 * i;
      const Vector3 rho = xi.template segment<3>(idx);
      x.col(i) = local.Jacobian().applyLeft(rho);
    }
  }

  return MakeReturn(ExtendedPose3(R, x));
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::TangentVector
ExtendedPose3<K, Derived>::Logmap(const This& pose, ChartJacobian H) {
  const ExtendedPose3& poseBase = AsBase(pose);
  const Vector3 w = Rot3::Logmap(poseBase.R_);
  const so3::DexpFunctor local(w);

  TangentVector xi;
  if constexpr (K == Eigen::Dynamic)
    xi.resize(static_cast<Eigen::Index>(poseBase.dim()));
  xi.template head<3>() = w;
  const Eigen::Index k = static_cast<Eigen::Index>(poseBase.k());
  for (Eigen::Index i = 0; i < k; ++i) {
    const Eigen::Index idx = 3 + 3 * i;
    xi.template segment<3>(idx) =
        local.InvJacobian().applyLeft(poseBase.t_.col(i));
  }

  if (H) *H = LogmapDerivative(xi);
  return xi;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::Jacobian
ExtendedPose3<K, Derived>::AdjointMap() const {
  const Matrix3 R = R_.matrix();

  Jacobian adj;
  if constexpr (dimension == Eigen::Dynamic) {
    adj.setZero(dim(), dim());
  } else {
    adj.setZero();
  }

  adj.block(0, 0, 3, 3) = R;
  const Eigen::Index k = static_cast<Eigen::Index>(this->k());
  for (Eigen::Index i = 0; i < k; ++i) {
    const Eigen::Index idx = 3 + 3 * i;
    adj.block(idx, 0, 3, 3) = skewSymmetric(t_.col(i)) * R;
    adj.block(idx, idx, 3, 3) = R;
  }
  return adj;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::Jacobian
ExtendedPose3<K, Derived>::adjointMap(const TangentVector& xi) {
  const Matrix3 w_hat = skewSymmetric(xi(0), xi(1), xi(2));

  const Eigen::Index k = static_cast<Eigen::Index>(RuntimeK(xi));

  Jacobian adj;
  if constexpr (dimension == Eigen::Dynamic) {
    adj.setZero(3 + 3 * k, 3 + 3 * k);
  } else {
    adj.setZero();
  }

  adj.block(0, 0, 3, 3) = w_hat;
  for (Eigen::Index i = 0; i < k; ++i) {
    const Eigen::Index idx = 3 + 3 * i;
    adj.block(idx, 0, 3, 3) =
        skewSymmetric(xi(idx + 0), xi(idx + 1), xi(idx + 2));
    adj.block(idx, idx, 3, 3) = w_hat;
  }
  return adj;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::Jacobian
ExtendedPose3<K, Derived>::ExpmapDerivative(const TangentVector& xi) {
  Jacobian J;
  Expmap(xi, J);
  return J;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::Jacobian
ExtendedPose3<K, Derived>::LogmapDerivative(const TangentVector& xi) {
  const Vector3 w = xi.template head<3>();

  // Instantiate functor for Dexp-related operations:
  const so3::DexpFunctor local(w);

  const Eigen::Index k = static_cast<Eigen::Index>(RuntimeK(xi));
  const Matrix3 Rt = local.expmap().transpose();
  const Matrix3 Jw = Rot3::LogmapDerivative(w);

  Jacobian J;
  if constexpr (dimension == Eigen::Dynamic) {
    J.setZero(3 + 3 * k, 3 + 3 * k);
  } else {
    J.setZero();
  }

  J.block(0, 0, 3, 3) = Jw;
  for (Eigen::Index i = 0; i < k; ++i) {
    Matrix3 H_xi_w;
    const Eigen::Index idx = 3 + 3 * i;
    local.Jacobian().applyLeft(xi.template segment<3>(idx), H_xi_w);
    const Matrix3 Q = Rt * H_xi_w;
    J.block(idx, 0, 3, 3) = -Jw * Q * Jw;
    J.block(idx, idx, 3, 3) = Jw;
  }
  return J;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::Jacobian
ExtendedPose3<K, Derived>::LogmapDerivative(const This& pose) {
  return LogmapDerivative(Logmap(pose));
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::This
ExtendedPose3<K, Derived>::ChartAtOrigin::Retract(const TangentVector& xi,
                                                  ChartJacobian Hxi) {
  return ExtendedPose3::Expmap(xi, Hxi);
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::TangentVector
ExtendedPose3<K, Derived>::ChartAtOrigin::Local(const This& pose,
                                                ChartJacobian H) {
  return ExtendedPose3::Logmap(pose, H);
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::MatrixRep
ExtendedPose3<K, Derived>::matrix() const {
  MatrixRep M;
  if constexpr (matrixDim == Eigen::Dynamic) {
    const Eigen::Index k = static_cast<Eigen::Index>(this->k());
    const Eigen::Index n = 3 + k;
    M = MatrixRep::Identity(n, n);
  } else {
    M = MatrixRep::Identity();
  }
  M.template block<3, 3>(0, 0) = R_.matrix();
  M.block(0, 3, 3, static_cast<Eigen::Index>(this->k())) = t_;
  return M;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::LieAlgebra ExtendedPose3<K, Derived>::Hat(
    const TangentVector& xi) {
  const Eigen::Index k = static_cast<Eigen::Index>(RuntimeK(xi));
  LieAlgebra X;
  if constexpr (matrixDim == Eigen::Dynamic) {
    X.setZero(3 + k, 3 + k);
  } else {
    X.setZero();
  }
  X.block(0, 0, 3, 3) = skewSymmetric(xi(0), xi(1), xi(2));
  for (Eigen::Index i = 0; i < k; ++i) {
    const Eigen::Index idx = 3 + 3 * i;
    X.block(0, 3 + i, 3, 1) = xi.template segment<3>(idx);
  }
  return X;
}

template <int K, class Derived>
typename ExtendedPose3<K, Derived>::TangentVector
ExtendedPose3<K, Derived>::Vee(const LieAlgebra& X) {
  if (X.rows() != X.cols() || X.rows() < 3) {
    throw std::invalid_argument("ExtendedPose3::Vee: invalid matrix shape.");
  }

  const Eigen::Index k = [&]() -> Eigen::Index {
    if constexpr (K == Eigen::Dynamic) {
      return X.cols() - 3;
    } else {
      if (X.rows() != matrixDim) {
        throw std::invalid_argument(
            "ExtendedPose3::Vee: invalid matrix shape.");
      }
      return static_cast<Eigen::Index>(K);
    }
  }();

  TangentVector xi;
  if constexpr (dimension == Eigen::Dynamic) {
    xi.resize(3 + 3 * k);
    xi.setZero();
  } else {
    xi.setZero();
  }
  xi(0) = X(2, 1);
  xi(1) = X(0, 2);
  xi(2) = X(1, 0);
  for (Eigen::Index i = 0; i < k; ++i) {
    const Eigen::Index idx = 3 + 3 * i;
    xi.template segment<3>(idx) = X.template block<3, 1>(0, 3 + i);
  }
  return xi;
}

}  // namespace gtsam
