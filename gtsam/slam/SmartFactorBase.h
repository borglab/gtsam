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
template<class CAMERA, size_t D>
class SmartFactorBase: public NonlinearFactor {

protected:

  typedef typename CAMERA::Measurement Z;

  /**
   * 2D measurement and noise model for each of the m views
   * We keep a copy of measurements for I/O and computing the error.
   * The order is kept the same as the keys that we use to create the factor.
   */
  std::vector<Z> measured_;

  //SharedIsotropic noiseModel_;
  std::vector<SharedNoiseModel> noise_; ///< noise model used

  boost::optional<Pose3> body_P_sensor_; ///< The pose of the sensor in the body frame (one for all cameras)

  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension

  /// Definitions for blocks of F
  typedef Eigen::Matrix<double, ZDim, D> Matrix2D; // F
  typedef Eigen::Matrix<double, D, ZDim> MatrixD2; // F'
  typedef std::pair<Key, Matrix2D> KeyMatrix2D; // Fblocks
  typedef Eigen::Matrix<double, D, D> MatrixDD; // camera hessian block
  typedef Eigen::Matrix<double, ZDim, 3> Matrix23;
  typedef Eigen::Matrix<double, D, 1> VectorD;
  typedef Eigen::Matrix<double, ZDim, ZDim> Matrix2;

  /// shorthand for base class type
  typedef NonlinearFactor Base;

  /// shorthand for this class
  typedef SmartFactorBase<CAMERA, D> This;

public:

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
   * @param noise_i is the measurement noise
   */
  void add(const Z& measured_i, const Key& poseKey_i,
      const SharedNoiseModel& noise_i) {
    this->measured_.push_back(measured_i);
    this->keys_.push_back(poseKey_i);
    this->noise_.push_back(noise_i);
  }

  /**
   * Add a bunch of measurements, together with the camera keys and noises
   */
  void add(std::vector<Z>& measurements, std::vector<Key>& poseKeys,
      std::vector<SharedNoiseModel>& noises) {
    for (size_t i = 0; i < measurements.size(); i++) {
      this->measured_.push_back(measurements.at(i));
      this->keys_.push_back(poseKeys.at(i));
      this->noise_.push_back(noises.at(i));
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
      this->noise_.push_back(noise);
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
      this->noise_.push_back(noise);
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

  /** return the noise models */
  const std::vector<SharedNoiseModel>& noise() const {
    return noise_;
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
      noise_[k]->print("noise model = ");
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

  /// Calculate vector of re-projection errors, before applying noise model
  Vector reprojectionError(const Cameras& cameras, const Point3& point) const {

    // Project into CameraSet
    std::vector<Z> predicted;
    try {
      predicted = cameras.project(point);
    } catch (CheiralityException&) {
      std::cout << "reprojectionError: Cheirality exception " << std::endl;
      exit(EXIT_FAILURE); // TODO: throw exception
    }

    // Calculate vector of errors
    size_t nrCameras = cameras.size();
    Vector b(ZDim * nrCameras);
    for (size_t i = 0, row = 0; i < nrCameras; i++, row += ZDim) {
      Z e = predicted[i] - measured_.at(i);
      b.segment<ZDim>(row) = e.vector();
    }

    return b;
  }

  /// Calculate vector of re-projection errors, noise model applied
  Vector whitenedError(const Cameras& cameras, const Point3& point) const {
    Vector b = reprojectionError(cameras, point);
    size_t nrCameras = cameras.size();
    for (size_t i = 0, row = 0; i < nrCameras; i++, row += ZDim)
      b.segment<ZDim>(row) = noise_.at(i)->whiten(b.segment<ZDim>(row));
    return b;
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   * This is different from reprojectionError(cameras,point) as each point is whitened
   */
  double totalReprojectionError(const Cameras& cameras,
      const Point3& point) const {
    Vector b = reprojectionError(cameras, point);
    double overallError = 0;
    size_t nrCameras = cameras.size();
    for (size_t i = 0; i < nrCameras; i++)
      overallError += noise_.at(i)->distance(b.segment<ZDim>(i * ZDim));
    return 0.5 * overallError;
  }

  /**
   * Compute whitenedError, returning only the derivative E, i.e.,
   * the stacked ZDim*3 derivatives of project with respect to the point.
   * Assumes non-degenerate ! TODO explain this
   */
  Vector whitenedError(const Cameras& cameras, const Point3& point,
      Matrix& E) const {

    // Project into CameraSet, calculating derivatives
    std::vector<Z> predicted;
    try {
      using boost::none;
      predicted = cameras.project(point, none, E, none);
    } catch (CheiralityException&) {
      std::cout << "whitenedError(E): Cheirality exception " << std::endl;
      exit(EXIT_FAILURE); // TODO: throw exception
    }

    // if needed, whiten
    size_t m = keys_.size();
    Vector b(ZDim * m);
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {

      // Calculate error
      const Z& zi = measured_.at(i);
      Vector bi = (zi - predicted[i]).vector();

      // if needed, whiten
      SharedNoiseModel model = noise_.at(i);
      if (model) {
        // TODO: re-factor noiseModel to take any block/fixed vector/matrix
        Vector dummy;
        Matrix Ei = E.block<ZDim, 3>(row, 0);
        model->WhitenSystem(Ei, dummy);
        E.block<ZDim, 3>(row, 0) = Ei;
      }
      b.segment<ZDim>(row) = bi;
    }
    return b;
  }

  /**
   *  Compute F, E, and optionally H, where F and E are the stacked derivatives
   *  with respect to the cameras, point, and calibration, respectively.
   *  The value of cameras/point are passed as parameters.
   *  Returns error vector b
   *  TODO: the treatment of body_P_sensor_ is weird: the transformation
   *  is applied in the caller, but the derivatives are computed here.
   */
  Vector whitenedError(const Cameras& cameras, const Point3& point, Matrix& F,
      Matrix& E, boost::optional<Matrix&> G = boost::none) const {

    // Project into CameraSet, calculating derivatives
    std::vector<Z> predicted;
    try {
      predicted = cameras.project(point, F, E, G);
    } catch (CheiralityException&) {
      std::cout << "whitenedError(E,F): Cheirality exception " << std::endl;
      exit(EXIT_FAILURE); // TODO: throw exception
    }

    // Calculate error and whiten derivatives if needed
    size_t m = keys_.size();
    Vector b(ZDim * m);
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {

      // Calculate error
      const Z& zi = measured_.at(i);
      Vector bi = (zi - predicted[i]).vector();

      // if we have a sensor offset, correct camera derivatives
      if (body_P_sensor_) {
        // TODO: no simpler way ??
        const Pose3& pose_i = cameras[i].pose();
        Pose3 w_Pose_body = pose_i.compose(body_P_sensor_->inverse());
        Matrix66 J;
        Pose3 world_P_body = w_Pose_body.compose(*body_P_sensor_, J);
        F.block<ZDim, 6>(row, 0) *= J;
      }

      // if needed, whiten
      SharedNoiseModel model = noise_.at(i);
      if (model) {
        // TODO, refactor noiseModel so we can take blocks
        Matrix Fi = F.block<ZDim, 6>(row, 0);
        Matrix Ei = E.block<ZDim, 3>(row, 0);
        if (!G)
          model->WhitenSystem(Fi, Ei, bi);
        else {
          Matrix Gi = G->block<ZDim, D - 6>(row, 0);
          model->WhitenSystem(Fi, Ei, Gi, bi);
          G->block<ZDim, D - 6>(row, 0) = Gi;
        }
        F.block<ZDim, 6>(row, 0) = Fi;
        E.block<ZDim, 3>(row, 0) = Ei;
      }
      b.segment<ZDim>(row) = bi;
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
    whitenedError(cameras, point, E);
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
    // TODO: if D==6 we optimize only camera pose. That is fairly hacky!
    Matrix F, G;
    using boost::none;
    boost::optional<Matrix&> optionalG(G);
    b = whitenedError(cameras, point, F, E, D == 6 ? none : optionalG);

    // Now calculate f and divide up the F derivatives into Fblocks
    double f = 0.0;
    size_t m = keys_.size();
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {

      // Accumulate normalized error
      f += b.segment<ZDim>(row).squaredNorm();

      // Get piece of F and possibly G
      Matrix2D Fi;
      if (D == 6)
        Fi << F.block<ZDim, 6>(row, 0);
      else
        Fi << F.block<ZDim, 6>(row, 0), G.block<ZDim, D - 6>(row, 0);

      // Push it onto Fblocks
      Fblocks.push_back(KeyMatrix2D(keys_[i], Fi));
    }
    return f;
  }

  /// Create BIG block-diagonal matrix F from Fblocks
  static void FillDiagonalF(const std::vector<KeyMatrix2D>& Fblocks, Matrix& F) {
    size_t m = Fblocks.size();
    F.resize(ZDim * m, D * m);
    F.setZero();
    for (size_t i = 0; i < m; ++i)
      F.block<This::ZDim, D>(This::ZDim * i, D * i) = Fblocks.at(i).second;
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
  boost::shared_ptr<RegularHessianFactor<D> > createHessianFactor(
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

    return boost::make_shared < RegularHessianFactor<D> > (this->keys_, Gs, gs, f);
#else // we create directly a SymmetricBlockMatrix
    size_t n1 = D * numKeys + 1;
    std::vector<DenseIndex> dims(numKeys + 1); // this also includes the b term
    std::fill(dims.begin(), dims.end() - 1, D);
    dims.back() = 1;

    SymmetricBlockMatrix augmentedHessian(dims, Matrix::Zero(n1, n1)); // for 10 cameras, size should be (10*D+1 x 10*D+1)
    sparseSchurComplement(Fblocks, E, P, b, augmentedHessian); // augmentedHessian.matrix().block<D,D> (i1,i2) = ...
    augmentedHessian(numKeys, numKeys)(0, 0) = f;
    return boost::make_shared<RegularHessianFactor<D> >(this->keys_,
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

    Matrix H(D * numKeys, D * numKeys);
    Vector gs_vector;

    H.noalias() = F.transpose() * (F - (E * (P * (E.transpose() * F))));
    gs_vector.noalias() = F.transpose() * (b - (E * (P * (E.transpose() * b))));

    // Populate Gs and gs
    int GsCount2 = 0;
    for (DenseIndex i1 = 0; i1 < numKeys; i1++) { // for each camera
      DenseIndex i1D = i1 * D;
      gs.at(i1) = gs_vector.segment<D>(i1D);
      for (DenseIndex i2 = 0; i2 < numKeys; i2++) {
        if (i2 >= i1) {
          Gs.at(GsCount2) = H.block<D, D>(i1D, i2 * D);
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

      // D = (Dx2) * (2)
      // (augmentedHessian.matrix()).block<D,1> (i1,numKeys+1) = Fi1.transpose() * b.segment < 2 > (2 * i1); // F' * b
      augmentedHessian(i1, numKeys) = Fi1.transpose()
          * b.segment<ZDim>(ZDim * i1) // F' * b
      - Fi1.transpose() * (Ei1_P * (E.transpose() * b)); // D = (DxZDim) * (ZDimx3) * (3*ZDimm) * (ZDimm x 1)

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
        // D = (Dx2) * (2)
        gs.at(i1) = Fi1.transpose() * b.segment<ZDim>(ZDim * i1) // F' * b
        - Fi1.transpose() * (Ei1_P * (E.transpose() * b)); // D = (DxZDim) * (ZDimx3) * (3*ZDimm) * (ZDimm x 1)

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
    size_t aug_numKeys = (augmentedHessian.rows() - 1) / D; // all cameras in the group

    // Blockwise Schur complement
    for (size_t i1 = 0; i1 < numKeys; i1++) { // for each camera in the current factor

      const Matrix2D& Fi1 = Fblocks.at(i1).second;
      const Matrix23 Ei1_P = E.block<ZDim, 3>(ZDim * i1, 0) * P;

      // D = (DxZDim) * (ZDim)
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
      - Fi1.transpose() * (Ei1_P * (E.transpose() * b)); // D = (DxZDim) * (ZDimx3) * (3*ZDimm) * (ZDimm x 1)

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
    updateSparseSchurComplement(Fblocks, E, P, b, f, allKeys, augmentedHessian); // augmentedHessian.matrix().block<D,D> (i1,i2) = ...
  }

  /**
   * Return Jacobians as RegularImplicitSchurFactor with raw access
   */
  boost::shared_ptr<RegularImplicitSchurFactor<D> > createRegularImplicitSchurFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    typename boost::shared_ptr<RegularImplicitSchurFactor<D> > f(
        new RegularImplicitSchurFactor<D>());
    computeJacobians(f->Fblocks(), f->E(), f->b(), cameras, point);
    f->PointCovariance() = PointCov(f->E(), lambda, diagonalDamping);
    f->initKeys();
    return f;
  }

  /**
   * Return Jacobians as JacobianFactorQ
   */
  boost::shared_ptr<JacobianFactorQ<D, ZDim> > createJacobianQFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    std::vector<KeyMatrix2D> Fblocks;
    Matrix E;
    Vector b;
    computeJacobians(Fblocks, E, b, cameras, point);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);
    return boost::make_shared<JacobianFactorQ<D, ZDim> >(Fblocks, E, P, b);
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
    return boost::make_shared<JacobianFactorSVD<6, ZDim> >(Fblocks, Enull, b);
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
}
;

template<class CAMERA, size_t D>
const int SmartFactorBase<CAMERA, D>::ZDim;

} // \ namespace gtsam
