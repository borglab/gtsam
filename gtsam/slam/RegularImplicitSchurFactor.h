/**
 * @file    RegularImplicitSchurFactor.h
 * @brief   A subclass of GaussianFactor specialized to structureless SFM.
 * @author  Frank Dellaert
 * @author  Luca Carlone
 */

#pragma once

#include <gtsam/geometry/CameraSet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>

#include <iosfwd>
#include <map>
#include <string>
#include <vector>

namespace gtsam {

/**
 * RegularImplicitSchurFactor
 * 
 * A specialization of a GaussianFactor to structure-less SFM, which is very
 * fast in a conjugate gradient (CG) solver. Specifically, as measured in 
 * timeSchurFactors.cpp, it stays very fast for an increasing number of cameras.
 * The magic is in multiplyHessianAdd, which does the Hessian-vector multiply at 
 * the core of CG, and implements
 *    y += F'*alpha*(I - E*P*E')*F*x
 * where 
 *  - F is the 2mx6m Jacobian of the m 2D measurements wrpt m 6DOF poses
 *  - E is the 2mx3 Jacabian of the m 2D measurements wrpt a 3D point
 *  - P is the covariance on the point
 * The equation above implicitly executes the Schur complement by removing the
 * information E*P*E' from the Hessian. It is also very fast as we do not use 
 * the full 6m*6m F matrix, but rather only it's m 6x6 diagonal blocks.
 */
template<class CAMERA>
class RegularImplicitSchurFactor: public GaussianFactor {

public:
  typedef RegularImplicitSchurFactor This; ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

protected:

  // This factor is closely related to a CameraSet
  typedef CameraSet<CAMERA> Set;

  typedef typename CAMERA::Measurement Z;
  static const int D = traits<CAMERA>::dimension; ///< Camera dimension
  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension

  typedef Eigen::Matrix<double, ZDim, D> MatrixZD; ///< type of an F block
  typedef Eigen::Matrix<double, D, D> MatrixDD; ///< camera Hessian
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks;

  FBlocks FBlocks_; ///< All ZDim*D F blocks (one for each camera)
  const Matrix PointCovariance_; ///< the 3*3 matrix P = inv(E'E) (2*2 if degenerate)
  const Matrix E_; ///< The 2m*3 E Jacobian with respect to the point
  const Vector b_; ///< 2m-dimensional RHS vector

public:

  /// Constructor
  RegularImplicitSchurFactor() {
  }

  /// Construct from blocks of F, E, inv(E'*E), and RHS vector b

  /**
   * @brief Construct a new RegularImplicitSchurFactor object.
   * 
   * @param keys keys corresponding to cameras
   * @param Fs All ZDim*D F blocks (one for each camera)
   * @param E Jacobian of measurements wrpt point.
   * @param P point covariance matrix
   * @param b RHS vector
   */
  RegularImplicitSchurFactor(const KeyVector& keys, const FBlocks& Fs,
                             const Matrix& E, const Matrix& P, const Vector& b)
      : GaussianFactor(keys), FBlocks_(Fs), PointCovariance_(P), E_(E), b_(b) {}

  /// Destructor
  ~RegularImplicitSchurFactor() override {
  }

  const FBlocks& Fs() const {
    return FBlocks_;
  }

  const Matrix& E() const {
    return E_;
  }

  const Vector& b() const {
    return b_;
  }

  const Matrix& getPointCovariance() const {
    return PointCovariance_;
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << " RegularImplicitSchurFactor " << std::endl;
    Factor::print(s);
    for (size_t pos = 0; pos < size(); ++pos) {
      std::cout << "Fblock:\n" << FBlocks_[pos] << std::endl;
    }
    std::cout << "PointCovariance:\n" << PointCovariance_ << std::endl;
    std::cout << "E:\n" << E_ << std::endl;
    std::cout << "b:\n" << b_.transpose() << std::endl;
  }

  /// equals
  bool equals(const GaussianFactor& lf, double tol) const override {
    const This* f = dynamic_cast<const This*>(&lf);
    if (!f)
      return false;
    for (size_t k = 0; k < FBlocks_.size(); ++k) {
      if (keys_[k] != f->keys_[k])
        return false;
      if (!equal_with_abs_tol(FBlocks_[k], f->FBlocks_[k], tol))
        return false;
    }
    return equal_with_abs_tol(PointCovariance_, f->PointCovariance_, tol)
        && equal_with_abs_tol(E_, f->E_, tol)
        && equal_with_abs_tol(b_, f->b_, tol);
  }

  /// Degrees of freedom of camera
  DenseIndex getDim(const_iterator variable) const override {
    return D;
  }

  void updateHessian(const KeyVector& keys,
                         SymmetricBlockMatrix* info) const override {
    throw std::runtime_error(
        "RegularImplicitSchurFactor::updateHessian non implemented");
  }
  Matrix augmentedJacobian() const override {
    throw std::runtime_error(
        "RegularImplicitSchurFactor::augmentedJacobian non implemented");
    return Matrix();
  }
  std::pair<Matrix, Vector> jacobian() const override {
    throw std::runtime_error(
        "RegularImplicitSchurFactor::jacobian non implemented");
    return {Matrix(), Vector()};
  }

  /// *Compute* full augmented information matrix
  Matrix augmentedInformation() const override {
    // Do the Schur complement
    SymmetricBlockMatrix augmentedHessian =
        Set::SchurComplement(FBlocks_, E_, b_);
    return augmentedHessian.selfadjointView();
  }

  /// *Compute* full information matrix
  Matrix information() const override {
    Matrix augmented = augmentedInformation();
    int m = this->keys_.size();
    size_t M = D * m;
    return augmented.block(0, 0, M, M);
  }

  /// Using the base method
  using GaussianFactor::hessianDiagonal;

  /// Add the diagonal of the Hessian for this factor to existing VectorValues
  void hessianDiagonalAdd(VectorValues &d) const override {
    // diag(Hessian) = diag(F' * (I - E * PointCov * E') * F);
    for (size_t k = 0; k < size(); ++k) { // for each camera
      Key j = keys_[k];

      // Calculate Fj'*Ej for the current camera (observing a single point)
      // D x 3 = (D x ZDim) * (ZDim x 3)
      const MatrixZD& Fj = FBlocks_[k];
      Eigen::Matrix<double, D, 3> FtE = Fj.transpose()
                                        * E_.block<ZDim, 3>(ZDim * k, 0);

      Eigen::Matrix<double, D, 1> dj;
      for (int k = 0; k < D; ++k) { // for each diagonal element of the camera hessian
        // Vector column_k_Fj = Fj.col(k);
        dj(k) = Fj.col(k).squaredNorm(); // dot(column_k_Fj, column_k_Fj);
        // Vector column_k_FtE = FtE.row(k);
        // (1 x 1) = (1 x 3) * (3 * 3) * (3 x 1)
        dj(k) -= FtE.row(k) * PointCovariance_ * FtE.row(k).transpose();
      }

      auto result = d.emplace(j, dj);
      if(!result.second) {
        result.first->second += dj;
      }
    }
  }

  /**
   * @brief add the contribution of this factor to the diagonal of the hessian
   * d(output) = d(input) + deltaHessianFactor
   */
  void hessianDiagonal(double* d) const override {
    // diag(Hessian) = diag(F' * (I - E * PointCov * E') * F);
    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, D, 1> DVector;
    typedef Eigen::Map<DVector> DMap;

    for (size_t pos = 0; pos < size(); ++pos) { // for each camera in the factor
      Key j = keys_[pos];

      // Calculate Fj'*Ej for the current camera (observing a single point)
      // D x 3 = (D x ZDim) * (ZDim x 3)
      const MatrixZD& Fj = FBlocks_[pos];
      Eigen::Matrix<double, D, 3> FtE = Fj.transpose()
          * E_.block<ZDim, 3>(ZDim * pos, 0);

      DVector dj;
      for (int k = 0; k < D; ++k) { // for each diagonal element of the camera hessian
        dj(k) = Fj.col(k).squaredNorm();
        // (1 x 1) = (1 x 3) * (3 * 3) * (3 x 1)
        dj(k) -= FtE.row(k) * PointCovariance_ * FtE.row(k).transpose();
      }
      DMap(d + D * j) += dj;
    }
  }

  /// Return the block diagonal of the Hessian for this factor
  std::map<Key, Matrix> hessianBlockDiagonal() const override {
    std::map<Key, Matrix> blocks;
    // F'*(I - E*P*E')*F
    for (size_t pos = 0; pos < size(); ++pos) {
      Key j = keys_[pos];
      // F'*F - F'*E*P*E'*F  e.g. (9*2)*(2*9) - (9*2)*(2*3)*(3*3)*(3*2)*(2*9)
      const MatrixZD& Fj = FBlocks_[pos];
      //      Eigen::Matrix<double, D, 3> FtE = Fj.transpose()
      //          * E_.block<ZDim, 3>(ZDim * pos, 0);
      //      blocks[j] = Fj.transpose() * Fj
      //          - FtE * PointCovariance_ * FtE.transpose();

      const Matrix23& Ej = E_.block<ZDim, 3>(ZDim * pos, 0);
      blocks[j] = Fj.transpose()
          * (Fj - Ej * PointCovariance_ * Ej.transpose() * Fj);

      // F'*(I - E*P*E')*F, TODO: this should work, but it does not :-(
      //      static const Eigen::Matrix<double, ZDim, ZDim> I2 = eye(ZDim);
      //      Matrix2 Q = //
      //          I2 - E_.block<ZDim, 3>(ZDim * pos, 0) * PointCovariance_ * E_.block<ZDim, 3>(ZDim * pos, 0).transpose();
      //      blocks[j] = Fj.transpose() * Q * Fj;
    }
    return blocks;
  }

  GaussianFactor::shared_ptr clone() const override {
    return std::make_shared<RegularImplicitSchurFactor<CAMERA> >(keys_,
        FBlocks_, PointCovariance_, E_, b_);
    throw std::runtime_error(
        "RegularImplicitSchurFactor::clone non implemented");
  }

  GaussianFactor::shared_ptr negate() const override {
    return std::make_shared<RegularImplicitSchurFactor<CAMERA> >(keys_,
        FBlocks_, PointCovariance_, E_, b_);
    throw std::runtime_error(
        "RegularImplicitSchurFactor::negate non implemented");
  }

  // Raw Vector version of y += F'*alpha*(I - E*P*E')*F*x, for testing
  static
  void multiplyHessianAdd(const Matrix& F, const Matrix& E,
      const Matrix& PointCovariance, double alpha, const Vector& x, Vector& y) {
    Vector e1 = F * x;
    Vector d1 = E.transpose() * e1;
    Vector d2 = PointCovariance * d1;
    Vector e2 = E * d2;
    Vector e3 = alpha * (e1 - e2);
    y += F.transpose() * e3;
  }

  typedef std::vector<Vector2, Eigen::aligned_allocator<Vector2>> Error2s;

  /**
   * @brief Calculate corrected error Q*(e-ZDim*b) = (I - E*P*E')*(e-ZDim*b)
   */
  void projectError2(const Error2s& e1, Error2s& e2) const {

    // d1 = E.transpose() * (e1-ZDim*b) = (3*2m)*2m
    Vector3 d1;
    d1.setZero();
    for (size_t k = 0; k < size(); k++)
      d1 += E_.block<ZDim, 3>(ZDim * k, 0).transpose()
          * (e1[k] - ZDim * b_.segment<ZDim>(k * ZDim));

    // d2 = E.transpose() * e1 = (3*2m)*2m
    Vector3 d2 = PointCovariance_ * d1;

    // e3 = alpha*(e1 - E*d2) = 1*[2m-(2m*3)*3]
    for (size_t k = 0; k < size(); k++)
      e2[k] = e1[k] - ZDim * b_.segment<ZDim>(k * ZDim)
          - E_.block<ZDim, 3>(ZDim * k, 0) * d2;
  }

  /*
   * This definition matches the linearized error in the Hessian Factor:
   * LinError(x) = x'*H*x - 2*x'*eta + f
   * with:
   * H   = F' * (I-E'*P*E) * F = F' * Q * F
   * eta = F' * (I-E'*P*E) * b = F' * Q * b
   * f = nonlinear error
   * (x'*H*x - 2*x'*eta + f) = x'*F'*Q*F*x - 2*x'*F'*Q *b + f = x'*F'*Q*(F*x - 2*b) + f
   */
  double error(const VectorValues& x) const override {

    // resize does not do malloc if correct size
    e1.resize(size());
    e2.resize(size());

    // e1 = F * x - b = (2m*dm)*dm
    for (size_t k = 0; k < size(); ++k)
      e1[k] = FBlocks_[k] * x.at(keys_[k]);
    projectError2(e1, e2);

    double result = 0;
    for (size_t k = 0; k < size(); ++k)
      result += dot(e1[k], e2[k]);

    double f = b_.squaredNorm();
    return 0.5 * (result + f);
  }

  // needed to be GaussianFactor - (I - E*P*E')*(F*x - b)
  // This is wrong and does not match the definition in Hessian,
  // but it matches the definition of the Jacobian factor (JF)
  double errorJF(const VectorValues& x) const {

    // resize does not do malloc if correct size
    e1.resize(size());
    e2.resize(size());

    // e1 = F * x - b = (2m*dm)*dm
    for (size_t k = 0; k < size(); ++k)
      e1[k] = FBlocks_[k] * x.at(keys_[k]) - b_.segment<ZDim>(k * ZDim);
    projectError(e1, e2);

    double result = 0;
    for (size_t k = 0; k < size(); ++k)
      result += dot(e2[k], e2[k]);

    // std::cout << "implicitFactor::error result " << result << std::endl;
    return 0.5 * result;
  }
  /**
   * @brief Calculate corrected error Q*e = (I - E*P*E')*e
   */
  void projectError(const Error2s& e1, Error2s& e2) const {

    // d1 = E.transpose() * e1 = (3*2m)*2m
    Vector3 d1;
    d1.setZero();
    for (size_t k = 0; k < size(); k++)
      d1 += E_.block<ZDim, 3>(ZDim * k, 0).transpose() * e1[k];

    // d2 = E.transpose() * e1 = (3*2m)*2m
    Vector3 d2 = PointCovariance_ * d1;

    // e3 = alpha*(e1 - E*d2) = 1*[2m-(2m*3)*3]
    for (size_t k = 0; k < size(); k++)
      e2[k] = e1[k] - E_.block<ZDim, 3>(ZDim * k, 0) * d2;
  }

  /// Scratch space for multiplyHessianAdd
  mutable Error2s e1, e2;

  /**
   * @brief double* Hessian-vector multiply, i.e. y += F'*alpha*(I - E*P*E')*F*x
   * RAW memory access! Assumes keys start at 0 and go to M-1, and x and and y are laid out that way
   */
  void multiplyHessianAdd(double alpha, const double* x, double* y) const {

    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, D, 1> DVector;
    typedef Eigen::Map<DVector> DMap;
    typedef Eigen::Map<const DVector> ConstDMap;

    // resize does not do malloc if correct size
    e1.resize(size());
    e2.resize(size());

    // e1 = F * x = (2m*dm)*dm
    for (size_t k = 0; k < size(); ++k) {
      Key key = keys_[k];
      e1[k] = FBlocks_[k] * ConstDMap(x + D * key);
    }

    projectError(e1, e2);

    // y += F.transpose()*e2 = (2d*2m)*2m
    for (size_t k = 0; k < size(); ++k) {
      Key key = keys_[k];
      DMap(y + D * key) += FBlocks_[k].transpose() * alpha * e2[k];
    }
  }

  void multiplyHessianAdd(double alpha, const double* x, double* y,
      std::vector<size_t> keys) const {
  }

  /**
   * @brief Hessian-vector multiply, i.e. y += F'*alpha*(I - E*P*E')*F*x
   */
  void multiplyHessianAdd(double alpha, const VectorValues& x,
      VectorValues& y) const override {

    // resize does not do malloc if correct size
    e1.resize(size());
    e2.resize(size());

    // e1 = F * x = (2m*dm)*dm
    for (size_t k = 0; k < size(); ++k)
      e1[k] = FBlocks_[k] * x.at(keys_[k]);

    projectError(e1, e2);

    // y += F.transpose()*e2 = (2d*2m)*2m
    for (size_t k = 0; k < size(); ++k) {
      Key key = keys_[k];
      static const Vector empty;
      std::pair<VectorValues::iterator, bool> it = y.tryInsert(key, empty);
      Vector& yi = it.first->second;
      // Create the value as a zero vector if it does not exist.
      if (it.second)
        yi = Vector::Zero(FBlocks_[k].cols());
      yi += FBlocks_[k].transpose() * alpha * e2[k];
    }
  }

  /**
   * @brief Dummy version to measure overhead of key access
   */
  void multiplyHessianDummy(double alpha, const VectorValues& x,
      VectorValues& y) const {

    for (size_t k = 0; k < size(); ++k) {
      static const Vector empty;
      Key key = keys_[k];
      std::pair<VectorValues::iterator, bool> it = y.tryInsert(key, empty);
      Vector& yi = it.first->second;
      yi = x.at(key);
    }
  }

  /**
   * Calculate gradient, which is -F'Q*b, see paper
   */
  VectorValues gradientAtZero() const override {
    // calculate Q*b
    e1.resize(size());
    e2.resize(size());
    for (size_t k = 0; k < size(); k++)
      e1[k] = b_.segment<ZDim>(ZDim * k);
    projectError(e1, e2);

    // g = F.transpose()*e2
    VectorValues g;
    for (size_t k = 0; k < size(); ++k) {
      Key key = keys_[k];
      g.insert(key, -FBlocks_[k].transpose() * e2[k]);
    }

    // return it
    return g;
  }

  /**
   * Calculate gradient, which is -F'Q*b, see paper - RAW MEMORY ACCESS
   */
  void gradientAtZero(double* d) const override {

    // Use eigen magic to access raw memory
    typedef Eigen::Matrix<double, D, 1> DVector;
    typedef Eigen::Map<DVector> DMap;

    // calculate Q*b
    e1.resize(size());
    e2.resize(size());
    for (size_t k = 0; k < size(); k++)
      e1[k] = b_.segment<ZDim>(ZDim * k);
    projectError(e1, e2);

    for (size_t k = 0; k < size(); ++k) { // for each camera in the factor
      Key j = keys_[k];
      DMap(d + D * j) += -FBlocks_[k].transpose() * e2[k];
    }
  }

  /// Gradient wrt a key at any values
  Vector gradient(Key key, const VectorValues& x) const override {
    throw std::runtime_error(
        "gradient for RegularImplicitSchurFactor is not implemented yet");
  }

};
// end class RegularImplicitSchurFactor

template<class CAMERA>
const int RegularImplicitSchurFactor<CAMERA>::D;

template<class CAMERA>
const int RegularImplicitSchurFactor<CAMERA>::ZDim;

// traits
template<class CAMERA> struct traits<RegularImplicitSchurFactor<CAMERA> > : public Testable<
    RegularImplicitSchurFactor<CAMERA> > {
};

}

