/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   CameraSet.h
 * @brief  Base class to create smart factors on poses or cameras
 * @author Frank Dellaert
 * @date   Feb 19, 2015
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/CalibratedCamera.h>  // for Cheirality exception
#include <gtsam/base/Testable.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/inference/Key.h>
#include <vector>

namespace gtsam {

/**
 * @brief A set of cameras, all with their own calibration
 */
template<class CAMERA>
class CameraSet : public std::vector<CAMERA, Eigen::aligned_allocator<CAMERA> > {

protected:

  /**
   * 2D measurement and noise model for each of the m views
   * The order is kept the same as the keys that we use to create the factor.
   */
  typedef typename CAMERA::Measurement Z;
  typedef typename CAMERA::MeasurementVector ZVector;

  static const int D = traits<CAMERA>::dimension; ///< Camera dimension
  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension

  /// Make a vector of re-projection errors
  static Vector ErrorVector(const ZVector& predicted,
      const ZVector& measured) {

    // Check size
    size_t m = predicted.size();
    if (measured.size() != m)
      throw std::runtime_error("CameraSet::errors: size mismatch");

    // Project and fill error vector
    Vector b(ZDim * m);
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {
      Vector bi = traits<Z>::Local(measured[i], predicted[i]);
      if(ZDim==3 && std::isnan(bi(1))){ // if it is a stereo point and the right pixel is missing (nan)
        bi(1) = 0;
      }
      b.segment<ZDim>(row) = bi;
    }
    return b;
  }

public:

  /// Definitions for blocks of F
  typedef Eigen::Matrix<double, ZDim, D> MatrixZD;
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks;

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  virtual void print(const std::string& s = "") const {
    std::cout << s << "CameraSet, cameras = \n";
    for (size_t k = 0; k < this->size(); ++k)
      this->at(k).print(s);
  }

  /// equals
  bool equals(const CameraSet& p, double tol = 1e-9) const {
    if (this->size() != p.size())
      return false;
    bool camerasAreEqual = true;
    for (size_t i = 0; i < this->size(); i++) {
      if (this->at(i).equals(p.at(i), tol) == false)
        camerasAreEqual = false;
      break;
    }
    return camerasAreEqual;
  }

  /**
   * Project a point (possibly Unit3 at infinity), with derivatives
   * Note that F is a sparse block-diagonal matrix, so instead of a large dense
   * matrix this function returns the diagonal blocks.
   * throws CheiralityException
   */
  template<class POINT>
  ZVector project2(const POINT& point, //
      boost::optional<FBlocks&> Fs = boost::none, //
      boost::optional<Matrix&> E = boost::none) const {

    static const int N = FixedDimension<POINT>::value;

    // Allocate result
    size_t m = this->size();
    ZVector z;
    z.reserve(m);

    // Allocate derivatives
    if (E) E->resize(ZDim * m, N);
    if (Fs) Fs->resize(m);

    // Project and fill derivatives
    for (size_t i = 0; i < m; i++) {
      MatrixZD Fi;
      Eigen::Matrix<double, ZDim, N> Ei;
      z.emplace_back(this->at(i).project2(point, Fs ? &Fi : 0, E ? &Ei : 0));
      if (Fs) (*Fs)[i] = Fi;
      if (E) E->block<ZDim, N>(ZDim * i, 0) = Ei;
    }

    return z;
  }

  /// Calculate vector [project2(point)-z] of re-projection errors
  template<class POINT>
  Vector reprojectionError(const POINT& point, const ZVector& measured,
      boost::optional<FBlocks&> Fs = boost::none, //
      boost::optional<Matrix&> E = boost::none) const {
    return ErrorVector(project2(point, Fs, E), measured);
  }

  /**
   * Do Schur complement, given Jacobian as Fs,E,P, return SymmetricBlockMatrix
   * G = F' * F - F' * E * P * E' * F
   * g = F' * (b - E * P * E' * b)
   * Fixed size version
   */
  template<int N> // N = 2 or 3
  static SymmetricBlockMatrix SchurComplement(const FBlocks& Fs,
      const Matrix& E, const Eigen::Matrix<double, N, N>& P, const Vector& b) {

    // a single point is observed in m cameras
    size_t m = Fs.size();

    // Create a SymmetricBlockMatrix
    size_t M1 = D * m + 1;
    std::vector<DenseIndex> dims(m + 1); // this also includes the b term
    std::fill(dims.begin(), dims.end() - 1, D);
    dims.back() = 1;
    SymmetricBlockMatrix augmentedHessian(dims, Matrix::Zero(M1, M1));

    // Blockwise Schur complement
    for (size_t i = 0; i < m; i++) { // for each camera

      const MatrixZD& Fi = Fs[i];
      const auto FiT = Fi.transpose();
      const Eigen::Matrix<double, ZDim, N> Ei_P = //
          E.block(ZDim * i, 0, ZDim, N) * P;

      // D = (Dx2) * ZDim
      augmentedHessian.setOffDiagonalBlock(i, m, FiT * b.segment<ZDim>(ZDim * i) // F' * b
      - FiT * (Ei_P * (E.transpose() * b))); // D = (DxZDim) * (ZDimx3) * (N*ZDimm) * (ZDimm x 1)

      // (DxD) = (DxZDim) * ( (ZDimxD) - (ZDimx3) * (3xZDim) * (ZDimxD) )
      augmentedHessian.setDiagonalBlock(i, FiT
          * (Fi - Ei_P * E.block(ZDim * i, 0, ZDim, N).transpose() * Fi));

      // upper triangular part of the hessian
      for (size_t j = i + 1; j < m; j++) { // for each camera
        const MatrixZD& Fj = Fs[j];

        // (DxD) = (Dx2) * ( (2x2) * (2xD) )
        augmentedHessian.setOffDiagonalBlock(i, j, -FiT
            * (Ei_P * E.block(ZDim * j, 0, ZDim, N).transpose() * Fj));
      }
    } // end of for over cameras

    augmentedHessian.diagonalBlock(m)(0, 0) += b.squaredNorm();
    return augmentedHessian;
  }

  /// Computes Point Covariance P, with lambda parameter
  template<int N> // N = 2 or 3
  static void ComputePointCovariance(Eigen::Matrix<double, N, N>& P,
      const Matrix& E, double lambda, bool diagonalDamping = false) {

    Matrix EtE = E.transpose() * E;

    if (diagonalDamping) { // diagonal of the hessian
      EtE.diagonal() += lambda * EtE.diagonal();
    } else {
      DenseIndex n = E.cols();
      EtE += lambda * Eigen::MatrixXd::Identity(n, n);
    }

    P = (EtE).inverse();
  }

  /// Computes Point Covariance P, with lambda parameter, dynamic version
  static Matrix PointCov(const Matrix& E, const double lambda = 0.0,
      bool diagonalDamping = false) {
    if (E.cols() == 2) {
      Matrix2 P2;
      ComputePointCovariance<2>(P2, E, lambda, diagonalDamping);
      return P2;
    } else {
      Matrix3 P3;
      ComputePointCovariance<3>(P3, E, lambda, diagonalDamping);
      return P3;
    }
  }

  /**
   * Do Schur complement, given Jacobian as Fs,E,P, return SymmetricBlockMatrix
   * Dynamic version
   */
  static SymmetricBlockMatrix SchurComplement(const FBlocks& Fblocks,
      const Matrix& E, const Vector& b, const double lambda = 0.0,
      bool diagonalDamping = false) {
    if (E.cols() == 2) {
      Matrix2 P;
      ComputePointCovariance<2>(P, E, lambda, diagonalDamping);
      return SchurComplement<2>(Fblocks, E, P, b);
    } else {
      Matrix3 P;
      ComputePointCovariance<3>(P, E, lambda, diagonalDamping);
      return SchurComplement<3>(Fblocks, E, P, b);
    }
  }

  /**
   * Applies Schur complement (exploiting block structure) to get a smart factor on cameras,
   * and adds the contribution of the smart factor to a pre-allocated augmented Hessian.
   */
  template<int N> // N = 2 or 3
  static void UpdateSchurComplement(const FBlocks& Fs, const Matrix& E,
      const Eigen::Matrix<double, N, N>& P, const Vector& b,
      const KeyVector& allKeys, const KeyVector& keys,
      /*output ->*/SymmetricBlockMatrix& augmentedHessian) {

    assert(keys.size()==Fs.size());
    assert(keys.size()<=allKeys.size());

    FastMap<Key, size_t> KeySlotMap;
    for (size_t slot = 0; slot < allKeys.size(); slot++)
      KeySlotMap.insert(std::make_pair(allKeys[slot], slot));

    // Schur complement trick
    // G = F' * F - F' * E * P * E' * F
    // g = F' * (b - E * P * E' * b)

    // a single point is observed in m cameras
    size_t m = Fs.size(); // cameras observing current point
    size_t M = (augmentedHessian.rows() - 1) / D; // all cameras in the group
    assert(allKeys.size()==M);

    // Blockwise Schur complement
    for (size_t i = 0; i < m; i++) { // for each camera in the current factor

      const MatrixZD& Fi = Fs[i];
      const auto FiT = Fi.transpose();
      const Eigen::Matrix<double, 2, N> Ei_P = E.template block<ZDim, N>(
          ZDim * i, 0) * P;

      // D = (DxZDim) * (ZDim)
      // allKeys are the list of all camera keys in the group, e.g, (1,3,4,5,7)
      // we should map those to a slot in the local (grouped) hessian (0,1,2,3,4)
      // Key cameraKey_i = this->keys_[i];
      DenseIndex aug_i = KeySlotMap.at(keys[i]);

      // information vector - store previous vector
      // vectorBlock = augmentedHessian(aug_i, aug_m).knownOffDiagonal();
      // add contribution of current factor
      augmentedHessian.updateOffDiagonalBlock(aug_i, M,
          FiT * b.segment<ZDim>(ZDim * i)      // F' * b
        - FiT * (Ei_P * (E.transpose() * b))); // D = (DxZDim) * (ZDimx3) * (N*ZDimm) * (ZDimm x 1)

      // (DxD) += (DxZDim) * ( (ZDimxD) - (ZDimx3) * (3xZDim) * (ZDimxD) )
      // add contribution of current factor
      // TODO(gareth): Eigen doesn't let us pass the expression. Call eval() for now...
      augmentedHessian.updateDiagonalBlock(aug_i,
         ((FiT * (Fi - Ei_P * E.template block<ZDim, N>(ZDim * i, 0).transpose() * Fi))).eval());

      // upper triangular part of the hessian
      for (size_t j = i + 1; j < m; j++) { // for each camera
        const MatrixZD& Fj = Fs[j];

        DenseIndex aug_j = KeySlotMap.at(keys[j]);

        // (DxD) = (DxZDim) * ( (ZDimxZDim) * (ZDimxD) )
        // off diagonal block - store previous block
        // matrixBlock = augmentedHessian(aug_i, aug_j).knownOffDiagonal();
        // add contribution of current factor
        augmentedHessian.updateOffDiagonalBlock(aug_i, aug_j,
                -FiT * (Ei_P * E.template block<ZDim, N>(ZDim * j, 0).transpose() * Fj));
      }
    } // end of for over cameras

    augmentedHessian.diagonalBlock(M)(0, 0) += b.squaredNorm();
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & (*this);
  }

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

template<class CAMERA>
const int CameraSet<CAMERA>::D;

template<class CAMERA>
const int CameraSet<CAMERA>::ZDim;

template<class CAMERA>
struct traits<CameraSet<CAMERA> > : public Testable<CameraSet<CAMERA> > {
};

template<class CAMERA>
struct traits<const CameraSet<CAMERA> > : public Testable<CameraSet<CAMERA> > {
};

} // \ namespace gtsam
