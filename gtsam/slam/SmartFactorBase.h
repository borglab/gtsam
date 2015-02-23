/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartFactorBase.h
 * @brief  Base class to create smart factors on poses or cameras
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/JacobianFactorQ.h>
#include <gtsam/slam/JacobianFactorSVD.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/RegularHessianFactor.h>
#include <gtsam/geometry/CameraSet.h>

#include <boost/optional.hpp>
#include <boost/make_shared.hpp>
#include <vector>

namespace gtsam {

/**
 * @brief  Base class for smart factors
 * This base class has no internal point, but it has a measurement, noise model
 * and an optional sensor pose.
 * This class mainly computes the derivatives and returns them as a variety of factors.
 * The methods take a Cameras argument, which should behave like PinholeCamera, and
 * the value of a point, which is kept in the base class.
 */
template<class CAMERA>
class SmartFactorBase: public NonlinearFactor {

private:
  typedef NonlinearFactor Base;
  typedef SmartFactorBase<CAMERA> This;
  typedef typename CAMERA::Measurement Z;

  /**
   * As of Feb 22, 2015, the noise model is the same for all measurements and
   * is isotropic. This allows for moving most calculations of Schur complement
   * etc to be moved to CameraSet very easily, and also agrees pragmatically
   * with what is normally done.
   */
  SharedIsotropic noiseModel_;

protected:

  /**
   * 2D measurement and noise model for each of the m views
   * We keep a copy of measurements for I/O and computing the error.
   * The order is kept the same as the keys that we use to create the factor.
   */
  std::vector<Z> measured_;

  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension
  static const int Dim = traits<CAMERA>::dimension; ///< Camera dimension

  // Definitions for block matrices used internally
  typedef Eigen::Matrix<double, Dim, ZDim> MatrixD2; // F'
  typedef Eigen::Matrix<double, Dim, Dim> MatrixDD; // camera hessian block
  typedef Eigen::Matrix<double, ZDim, 3> Matrix23;
  typedef Eigen::Matrix<double, Dim, 1> VectorD;
  typedef Eigen::Matrix<double, ZDim, ZDim> Matrix2;

  /// An optional sensor pose, in the body frame (one for all cameras)
  /// This seems to make sense for a CameraTrack, not for a CameraSet
  boost::optional<Pose3> body_P_sensor_;

  // check that noise model is isotropic and the same
  void maybeSetNoiseModel(const SharedNoiseModel& sharedNoiseModel) {
    if (!sharedNoiseModel)
      return;
    SharedIsotropic sharedIsotropic = boost::dynamic_pointer_cast<
        noiseModel::Isotropic>(sharedNoiseModel);
    if (!sharedIsotropic)
      throw std::runtime_error("SmartFactorBase: needs isotropic");
    if (!noiseModel_)
      noiseModel_ = sharedIsotropic;
    else if (!sharedIsotropic->equals(*noiseModel_))
      throw std::runtime_error(
          "SmartFactorBase: cannot add measurements with different noise model");
  }

public:

  // Definitions for blocks of F, externally visible
  typedef Eigen::Matrix<double, ZDim, Dim> Matrix2D; // F
  typedef std::pair<Key, Matrix2D> KeyMatrix2D; // Fblocks

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  typedef CameraSet<CAMERA> Cameras;

  /**
   * Constructor
   * @param body_P_sensor is the transform from sensor to body frame (default identity)
   */
  SmartFactorBase(boost::optional<Pose3> body_P_sensor = boost::none) :
      body_P_sensor_(body_P_sensor) {
  }

  /** Virtual destructor */
  virtual ~SmartFactorBase() {
  }

  /**
   * Add a new measurement and pose key
   * @param measured_i is the 2m dimensional projection of a single landmark
   * @param poseKey is the index corresponding to the camera observing the landmark
   * @param sharedNoiseModel is the measurement noise
   */
  void add(const Z& measured_i, const Key& poseKey_i,
      const SharedNoiseModel& sharedNoiseModel) {
    this->measured_.push_back(measured_i);
    this->keys_.push_back(poseKey_i);
    maybeSetNoiseModel(sharedNoiseModel);
  }

  /**
   * Add a bunch of measurements, together with the camera keys and noises
   */
  void add(std::vector<Z>& measurements, std::vector<Key>& poseKeys,
      std::vector<SharedNoiseModel>& noises) {
    for (size_t i = 0; i < measurements.size(); i++) {
      this->measured_.push_back(measurements.at(i));
      this->keys_.push_back(poseKeys.at(i));
      maybeSetNoiseModel(noises.at(i));
    }
  }

  /**
   * Add a bunch of measurements and uses the same noise model for all of them
   */
  void add(std::vector<Z>& measurements, std::vector<Key>& poseKeys,
      const SharedNoiseModel& noise) {
    for (size_t i = 0; i < measurements.size(); i++) {
      this->measured_.push_back(measurements.at(i));
      this->keys_.push_back(poseKeys.at(i));
      maybeSetNoiseModel(noise);
    }
  }

  /**
   * Adds an entire SfM_track (collection of cameras observing a single point).
   * The noise is assumed to be the same for all measurements
   */
  template<class SFM_TRACK>
  void add(const SFM_TRACK& trackToAdd, const SharedNoiseModel& noise) {
    for (size_t k = 0; k < trackToAdd.number_measurements(); k++) {
      this->measured_.push_back(trackToAdd.measurements[k].second);
      this->keys_.push_back(trackToAdd.measurements[k].first);
      maybeSetNoiseModel(noise);
    }
  }

  /// get the dimension (number of rows!) of the factor
  virtual size_t dim() const {
    return ZDim * this->measured_.size();
  }

  /** return the measurements */
  const std::vector<Z>& measured() const {
    return measured_;
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "SmartFactorBase, z = \n";
    for (size_t k = 0; k < measured_.size(); ++k) {
      std::cout << "measurement, p = " << measured_[k] << "\t";
      noiseModel_->print("noise model = ");
    }
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);

    bool areMeasurementsEqual = true;
    for (size_t i = 0; i < measured_.size(); i++) {
      if (this->measured_.at(i).equals(e->measured_.at(i), tol) == false)
        areMeasurementsEqual = false;
      break;
    }
    return e && Base::equals(p, tol) && areMeasurementsEqual
        && ((!body_P_sensor_ && !e->body_P_sensor_)
            || (body_P_sensor_ && e->body_P_sensor_
                && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /// Calculate vector of re-projection errors, noise model applied
  Vector whitenedErrors(const Cameras& cameras, const Point3& point) const {
    Vector b = cameras.reprojectionErrors(point, measured_);
    if (noiseModel_)
      noiseModel_->whitenInPlace(b);
    return b;
  }

  /// Calculate vector of re-projection errors, noise model applied
  // TODO: Unit3
  Vector whitenedErrorsAtInfinity(const Cameras& cameras,
      const Point3& point) const {
    Vector b = cameras.reprojectionErrorsAtInfinity(point, measured_);
    if (noiseModel_)
      noiseModel_->whitenInPlace(b);
    return b;
  }

  /** Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   * This is different from reprojectionError(cameras,point) as each point is whitened
   */
  double totalReprojectionError(const Cameras& cameras,
      const Point3& point) const {
    Vector b = whitenedErrors(cameras, point);
    return 0.5 * b.dot(b);
  }

  /// Version for infinity
  // TODO: Unit3
  double totalReprojectionErrorAtInfinity(const Cameras& cameras,
      const Point3& point) const {
    Vector b = whitenedErrorsAtInfinity(cameras, point);
    return 0.5 * b.dot(b);
  }

  /**
   *  Compute reprojection errors and derivatives
   *  TODO: the treatment of body_P_sensor_ is weird: the transformation
   *  is applied in the caller, but the derivatives are computed here.
   */
  Vector reprojectionErrors(const Cameras& cameras, const Point3& point,
      typename Cameras::FBlocks& F, Matrix& E) const {

    Vector b = cameras.reprojectionErrors(point, measured_, F, E);

    // Apply sensor chain rule if needed TODO: no simpler way ??
    if (body_P_sensor_) {
      size_t m = keys_.size();
      for (size_t i = 0; i < m; i++) {
        const Pose3& pose_i = cameras[i].pose();
        Pose3 w_Pose_body = pose_i.compose(body_P_sensor_->inverse());
        Matrix66 J;
        Pose3 world_P_body = w_Pose_body.compose(*body_P_sensor_, J);
        F[i].leftCols(6) *= J;
      }
    }

    return b;
  }

  /// Computes Point Covariance P from E
  static Matrix3 PointCov(Matrix& E) {
    return (E.transpose() * E).inverse();
  }

  /// Computes Point Covariance P, with lambda parameter
  static Matrix3 PointCov(Matrix& E, double lambda,
      bool diagonalDamping = false) {

    Matrix3 EtE = E.transpose() * E;

    if (diagonalDamping) { // diagonal of the hessian
      EtE(0, 0) += lambda * EtE(0, 0);
      EtE(1, 1) += lambda * EtE(1, 1);
      EtE(2, 2) += lambda * EtE(2, 2);
    } else {
      EtE(0, 0) += lambda;
      EtE(1, 1) += lambda;
      EtE(2, 2) += lambda;
    }

    return (EtE).inverse();
  }

  /// Assumes non-degenerate !
  void computeEP(Matrix& E, Matrix& P, const Cameras& cameras,
      const Point3& point) const {
    cameras.reprojectionErrors(point, measured_, boost::none, E);
    P = PointCov(E);
  }

  /**
   *  Compute F, E, and b (called below in both vanilla and SVD versions), where
   *  F is a vector of derivatives wrpt the cameras, and E the stacked derivatives
   *  with respect to the point. The value of cameras/point are passed as parameters.
   */
  double computeJacobians(std::vector<KeyMatrix2D>& Fblocks, Matrix& E,
      Vector& b, const Cameras& cameras, const Point3& point) const {

    // Project into Camera set and calculate derivatives
    typename Cameras::FBlocks F;
    b = reprojectionErrors(cameras, point, F, E);

    // Now calculate f and divide up the F derivatives into Fblocks
    double f = 0.0;
    size_t m = keys_.size();
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {

      // Accumulate normalized error
      f += b.segment<ZDim>(row).squaredNorm();

      // Push piece of F onto Fblocks with associated key
      Fblocks.push_back(KeyMatrix2D(keys_[i], F[i]));
    }
    return f;
  }

  /// Create BIG block-diagonal matrix F from Fblocks
  static void FillDiagonalF(const std::vector<KeyMatrix2D>& Fblocks,
      Matrix& F) {
    size_t m = Fblocks.size();
    F.resize(ZDim * m, Dim * m);
    F.setZero();
    for (size_t i = 0; i < m; ++i)
      F.block<This::ZDim, Dim>(This::ZDim * i, Dim * i) = Fblocks.at(i).second;
  }

  /**
   *  Compute F, E, and b, where F and E are the stacked derivatives
   *  with respect to the point. The value of cameras/point are passed as parameters.
   */
  double computeJacobians(Matrix& F, Matrix& E, Vector& b,
      const Cameras& cameras, const Point3& point) const {
    std::vector<KeyMatrix2D> Fblocks;
    double f = computeJacobians(Fblocks, E, b, cameras, point);
    FillDiagonalF(Fblocks, F);
    return f;
  }

  /// SVD version
  double computeJacobiansSVD(std::vector<KeyMatrix2D>& Fblocks, Matrix& Enull,
      Vector& b, const Cameras& cameras, const Point3& point) const {

    Matrix E;
    double f = computeJacobians(Fblocks, E, b, cameras, point);

    // Do SVD on A
    Eigen::JacobiSVD<Matrix> svd(E, Eigen::ComputeFullU);
    Vector s = svd.singularValues();
    size_t m = this->keys_.size();
    // Enull = zeros(ZDim * m, ZDim * m - 3);
    Enull = svd.matrixU().block(0, 3, ZDim * m, ZDim * m - 3); // last ZDim*m-3 columns

    return f;
  }

  /// Matrix version of SVD
  // TODO, there should not be a Matrix version, really
  double computeJacobiansSVD(Matrix& F, Matrix& Enull, Vector& b,
      const Cameras& cameras, const Point3& point) const {
    std::vector<KeyMatrix2D> Fblocks;
    double f = computeJacobiansSVD(Fblocks, Enull, b, cameras, point);
    FillDiagonalF(Fblocks, F);
    return f;
  }

  /**
   * Linearize returns a Hessianfactor that is an approximation of error(p)
   */
  boost::shared_ptr<RegularHessianFactor<Dim> > createHessianFactor(
      const Cameras& cameras, const Point3& point, const double lambda = 0.0,
      bool diagonalDamping = false) const {

    int numKeys = this->keys_.size();

    std::vector<KeyMatrix2D> Fblocks;
    Matrix E;
    Vector b;
    double f = computeJacobians(Fblocks, E, b, cameras, point);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);

//#define HESSIAN_BLOCKS // slower, as internally the Hessian factor will transform the blocks into SymmetricBlockMatrix
#ifdef HESSIAN_BLOCKS
    // Create structures for Hessian Factors
    std::vector < Matrix > Gs(numKeys * (numKeys + 1) / 2);
    std::vector < Vector > gs(numKeys);

    sparseSchurComplement(Fblocks, E, P, b, Gs, gs);
    // schurComplement(Fblocks, E, P, b, Gs, gs);

    //std::vector < Matrix > Gs2(Gs.begin(), Gs.end());
    //std::vector < Vector > gs2(gs.begin(), gs.end());

    return boost::make_shared < RegularHessianFactor<Dim> > (this->keys_, Gs, gs, f);
#else // we create directly a SymmetricBlockMatrix
    size_t n1 = Dim * numKeys + 1;
    std::vector<DenseIndex> dims(numKeys + 1); // this also includes the b term
    std::fill(dims.begin(), dims.end() - 1, Dim);
    dims.back() = 1;

    SymmetricBlockMatrix augmentedHessian(dims, Matrix::Zero(n1, n1)); // for 10 cameras, size should be (10*Dim+1 x 10*Dim+1)
    sparseSchurComplement(Fblocks, E, P, b, augmentedHessian); // augmentedHessian.matrix().block<Dim,Dim> (i1,i2) = ...
    augmentedHessian(numKeys, numKeys)(0, 0) = f;
    return boost::make_shared<RegularHessianFactor<Dim> >(this->keys_,
        augmentedHessian);
#endif
  }

  /**
   * Do Schur complement, given Jacobian as F,E,P.
   * Slow version - works on full matrices
   */
  void schurComplement(const std::vector<KeyMatrix2D>& Fblocks, const Matrix& E,
      const Matrix3& P, const Vector& b,
      /*output ->*/std::vector<Matrix>& Gs, std::vector<Vector>& gs) const {
    // Schur complement trick
    // Gs = F' * F - F' * E * inv(E'*E) * E' * F
    // gs = F' * (b - E * inv(E'*E) * E' * b)
    // This version uses full matrices

    int numKeys = this->keys_.size();

    /// Compute Full F ????
    Matrix F;
    FillDiagonalF(Fblocks, F);

    Matrix H(Dim * numKeys, Dim * numKeys);
    Vector gs_vector;

    H.noalias() = F.transpose() * (F - (E * (P * (E.transpose() * F))));
    gs_vector.noalias() = F.transpose() * (b - (E * (P * (E.transpose() * b))));

    // Populate Gs and gs
    int GsCount2 = 0;
    for (DenseIndex i1 = 0; i1 < numKeys; i1++) { // for each camera
      DenseIndex i1D = i1 * Dim;
      gs.at(i1) = gs_vector.segment<Dim>(i1D);
      for (DenseIndex i2 = 0; i2 < numKeys; i2++) {
        if (i2 >= i1) {
          Gs.at(GsCount2) = H.block<Dim, Dim>(i1D, i2 * Dim);
          GsCount2++;
        }
      }
    }
  }

  /**
   * Do Schur complement, given Jacobian as F,E,P, return SymmetricBlockMatrix
   * Fast version - works on with sparsity
   */
  void sparseSchurComplement(const std::vector<KeyMatrix2D>& Fblocks,
      const Matrix& E, const Matrix3& P /*Point Covariance*/, const Vector& b,
      /*output ->*/SymmetricBlockMatrix& augmentedHessian) const {
    // Schur complement trick
    // Gs = F' * F - F' * E * P * E' * F
    // gs = F' * (b - E * P * E' * b)

    // a single point is observed in numKeys cameras
    size_t numKeys = this->keys_.size();

    // Blockwise Schur complement
    for (size_t i1 = 0; i1 < numKeys; i1++) { // for each camera

      const Matrix2D& Fi1 = Fblocks.at(i1).second;
      const Matrix23 Ei1_P = E.block<ZDim, 3>(ZDim * i1, 0) * P;

      // Dim = (Dx2) * (2)
      // (augmentedHessian.matrix()).block<Dim,1> (i1,numKeys+1) = Fi1.transpose() * b.segment < 2 > (2 * i1); // F' * b
      augmentedHessian(i1, numKeys) = Fi1.transpose()
          * b.segment<ZDim>(ZDim * i1) // F' * b
      - Fi1.transpose() * (Ei1_P * (E.transpose() * b)); // Dim = (DxZDim) * (ZDimx3) * (3*ZDimm) * (ZDimm x 1)

      // (DxD) = (DxZDim) * ( (ZDimxD) - (ZDimx3) * (3xZDim) * (ZDimxD) )
      augmentedHessian(i1, i1) = Fi1.transpose()
          * (Fi1 - Ei1_P * E.block<ZDim, 3>(ZDim * i1, 0).transpose() * Fi1);

      // upper triangular part of the hessian
      for (size_t i2 = i1 + 1; i2 < numKeys; i2++) { // for each camera
        const Matrix2D& Fi2 = Fblocks.at(i2).second;

        // (DxD) = (Dx2) * ( (2x2) * (2xD) )
        augmentedHessian(i1, i2) = -Fi1.transpose()
            * (Ei1_P * E.block<ZDim, 3>(ZDim * i2, 0).transpose() * Fi2);
      }
    } // end of for over cameras
  }

  /**
   * Do Schur complement, given Jacobian as F,E,P, return Gs/gs
   * Fast version - works on with sparsity
   */
  void sparseSchurComplement(const std::vector<KeyMatrix2D>& Fblocks,
      const Matrix& E, const Matrix3& P /*Point Covariance*/, const Vector& b,
      /*output ->*/std::vector<Matrix>& Gs, std::vector<Vector>& gs) const {
    // Schur complement trick
    // Gs = F' * F - F' * E * P * E' * F
    // gs = F' * (b - E * P * E' * b)

    // a single point is observed in numKeys cameras
    size_t numKeys = this->keys_.size();

    int GsIndex = 0;
    // Blockwise Schur complement
    for (size_t i1 = 0; i1 < numKeys; i1++) { // for each camera
      // GsIndex points to the upper triangular blocks
      // 0  1  2  3  4
      // X  5  6  7  8
      // X  X  9 10 11
      // X  X  X 12 13
      // X  X  X X  14
      const Matrix2D& Fi1 = Fblocks.at(i1).second;

      const Matrix23 Ei1_P = E.block<ZDim, 3>(ZDim * i1, 0) * P;

      { // for i1 = i2
        // Dim = (Dx2) * (2)
        gs.at(i1) = Fi1.transpose() * b.segment<ZDim>(ZDim * i1) // F' * b
        - Fi1.transpose() * (Ei1_P * (E.transpose() * b)); // Dim = (DxZDim) * (ZDimx3) * (3*ZDimm) * (ZDimm x 1)

        // (DxD) = (DxZDim) * ( (ZDimxD) - (ZDimx3) * (3xZDim) * (ZDimxD) )
        Gs.at(GsIndex) = Fi1.transpose()
            * (Fi1 - Ei1_P * E.block<ZDim, 3>(ZDim * i1, 0).transpose() * Fi1);
        GsIndex++;
      }
      // upper triangular part of the hessian
      for (size_t i2 = i1 + 1; i2 < numKeys; i2++) { // for each camera
        const Matrix2D& Fi2 = Fblocks.at(i2).second;

        // (DxD) = (Dx2) * ( (2x2) * (2xD) )
        Gs.at(GsIndex) = -Fi1.transpose()
            * (Ei1_P * E.block<ZDim, 3>(ZDim * i2, 0).transpose() * Fi2);
        GsIndex++;
      }
    } // end of for over cameras
  }

  /**
   * Applies Schur complement (exploiting block structure) to get a smart factor on cameras,
   * and adds the contribution of the smart factor to a pre-allocated augmented Hessian.
   */
  void updateSparseSchurComplement(const std::vector<KeyMatrix2D>& Fblocks,
      const Matrix& E, const Matrix3& P /*Point Covariance*/, const Vector& b,
      const double f, const FastVector<Key> allKeys,
      /*output ->*/SymmetricBlockMatrix& augmentedHessian) const {
    // Schur complement trick
    // Gs = F' * F - F' * E * P * E' * F
    // gs = F' * (b - E * P * E' * b)

    MatrixDD matrixBlock;
    typedef SymmetricBlockMatrix::Block Block; ///< A block from the Hessian matrix

    FastMap<Key, size_t> KeySlotMap;
    for (size_t slot = 0; slot < allKeys.size(); slot++)
      KeySlotMap.insert(std::make_pair(allKeys[slot], slot));

    // a single point is observed in numKeys cameras
    size_t numKeys = this->keys_.size(); // cameras observing current point
    size_t aug_numKeys = (augmentedHessian.rows() - 1) / Dim; // all cameras in the group

    // Blockwise Schur complement
    for (size_t i1 = 0; i1 < numKeys; i1++) { // for each camera in the current factor

      const Matrix2D& Fi1 = Fblocks.at(i1).second;
      const Matrix23 Ei1_P = E.block<ZDim, 3>(ZDim * i1, 0) * P;

      // Dim = (DxZDim) * (ZDim)
      // allKeys are the list of all camera keys in the group, e.g, (1,3,4,5,7)
      // we should map those to a slot in the local (grouped) hessian (0,1,2,3,4)
      // Key cameraKey_i1 = this->keys_[i1];
      DenseIndex aug_i1 = KeySlotMap[this->keys_[i1]];

      // information vector - store previous vector
      // vectorBlock = augmentedHessian(aug_i1, aug_numKeys).knownOffDiagonal();
      // add contribution of current factor
      augmentedHessian(aug_i1, aug_numKeys) = augmentedHessian(aug_i1,
          aug_numKeys).knownOffDiagonal()
          + Fi1.transpose() * b.segment<ZDim>(ZDim * i1) // F' * b
      - Fi1.transpose() * (Ei1_P * (E.transpose() * b)); // Dim = (DxZDim) * (ZDimx3) * (3*ZDimm) * (ZDimm x 1)

      // (DxD) = (DxZDim) * ( (ZDimxD) - (ZDimx3) * (3xZDim) * (ZDimxD) )
      // main block diagonal - store previous block
      matrixBlock = augmentedHessian(aug_i1, aug_i1);
      // add contribution of current factor
      augmentedHessian(aug_i1, aug_i1) =
          matrixBlock
              + (Fi1.transpose()
                  * (Fi1
                      - Ei1_P * E.block<ZDim, 3>(ZDim * i1, 0).transpose() * Fi1));

      // upper triangular part of the hessian
      for (size_t i2 = i1 + 1; i2 < numKeys; i2++) { // for each camera
        const Matrix2D& Fi2 = Fblocks.at(i2).second;

        //Key cameraKey_i2 = this->keys_[i2];
        DenseIndex aug_i2 = KeySlotMap[this->keys_[i2]];

        // (DxD) = (DxZDim) * ( (ZDimxZDim) * (ZDimxD) )
        // off diagonal block - store previous block
        // matrixBlock = augmentedHessian(aug_i1, aug_i2).knownOffDiagonal();
        // add contribution of current factor
        augmentedHessian(aug_i1, aug_i2) =
            augmentedHessian(aug_i1, aug_i2).knownOffDiagonal()
                - Fi1.transpose()
                    * (Ei1_P * E.block<ZDim, 3>(ZDim * i2, 0).transpose() * Fi2);
      }
    } // end of for over cameras

    augmentedHessian(aug_numKeys, aug_numKeys)(0, 0) += f;
  }

  /**
   * Add the contribution of the smart factor to a pre-allocated Hessian,
   * using sparse linear algebra. More efficient than the creation of the
   * Hessian without preallocation of the SymmetricBlockMatrix
   */
  void updateAugmentedHessian(const Cameras& cameras, const Point3& point,
      const double lambda, bool diagonalDamping,
      SymmetricBlockMatrix& augmentedHessian,
      const FastVector<Key> allKeys) const {

    // int numKeys = this->keys_.size();

    std::vector<KeyMatrix2D> Fblocks;
    Matrix E;
    Vector b;
    double f = computeJacobians(Fblocks, E, b, cameras, point);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);
    updateSparseSchurComplement(Fblocks, E, P, b, f, allKeys, augmentedHessian); // augmentedHessian.matrix().block<Dim,Dim> (i1,i2) = ...
  }

  /**
   * Return Jacobians as RegularImplicitSchurFactor with raw access
   */
  boost::shared_ptr<RegularImplicitSchurFactor<Dim> > createRegularImplicitSchurFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    typename boost::shared_ptr<RegularImplicitSchurFactor<Dim> > f(
        new RegularImplicitSchurFactor<Dim>());
    computeJacobians(f->Fblocks(), f->E(), f->b(), cameras, point);
    f->PointCovariance() = PointCov(f->E(), lambda, diagonalDamping);
    f->initKeys();
    return f;
  }

  /**
   * Return Jacobians as JacobianFactorQ
   */
  boost::shared_ptr<JacobianFactorQ<Dim, ZDim> > createJacobianQFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    std::vector<KeyMatrix2D> Fblocks;
    Matrix E;
    Vector b;
    computeJacobians(Fblocks, E, b, cameras, point);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);
    return boost::make_shared<JacobianFactorQ<Dim, ZDim> >(Fblocks, E, P, b);
  }

  /**
   * Return Jacobians as JacobianFactor
   * TODO lambda is currently ignored
   */
  boost::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0) const {
    size_t numKeys = this->keys_.size();
    std::vector<KeyMatrix2D> Fblocks;
    Vector b;
    Matrix Enull(ZDim * numKeys, ZDim * numKeys - 3);
    computeJacobiansSVD(Fblocks, Enull, b, cameras, point);
    return boost::make_shared<JacobianFactorSVD<Dim, ZDim> >(Fblocks, Enull, b);
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};
// end class SmartFactorBase

// TODO: Why is this here?
template<class CAMERA>
const int SmartFactorBase<CAMERA>::ZDim;

} // \ namespace gtsam
