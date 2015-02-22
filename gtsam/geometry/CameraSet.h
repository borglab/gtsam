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
#include <gtsam/geometry/CalibratedCamera.h> // for Cheirality exception
#include <gtsam/base/Testable.h>
#include <vector>

namespace gtsam {

/**
 * @brief A set of cameras, all with their own calibration
 * Assumes that a camera is laid out as 6 Pose3 parameters then calibration
 */
template<class CAMERA>
class CameraSet: public std::vector<CAMERA> {

protected:

  /**
   * 2D measurement and noise model for each of the m views
   * The order is kept the same as the keys that we use to create the factor.
   */
  typedef typename CAMERA::Measurement Z;

  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension
  static const int Dim = traits<CAMERA>::dimension; ///< Camera dimension

  /// Make a vector of re-projection errors
  static Vector ErrorVector(const std::vector<Z>& predicted,
      const std::vector<Z>& measured) {

    // Check size
    size_t m = predicted.size();
    if (measured.size() != m)
      throw std::runtime_error("CameraSet::errors: size mismatch");

    // Project and fill derivatives
    Vector b(ZDim * m);
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {
      Z e = predicted[i] - measured[i];
      b.segment<ZDim>(row) = e.vector();
    }
    return b;
  }

public:

  /// Definitions for blocks of F
  typedef Eigen::Matrix<double, ZDim, Dim> MatrixZD; // F
  typedef std::pair<Key, MatrixZD> FBlock; // Fblocks

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "") const {
    std::cout << s << "CameraSet, cameras = \n";
    for (size_t k = 0; k < this->size(); ++k)
      this->at(k).print();
  }

  /// equals
  virtual bool equals(const CameraSet& p, double tol = 1e-9) const {
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
   * Project a point, with derivatives in this, point, and calibration
   * throws CheiralityException
   */
  std::vector<Z> project(const Point3& point, //
      boost::optional<Matrix&> F = boost::none, //
      boost::optional<Matrix&> E = boost::none, //
      boost::optional<Matrix&> H = boost::none) const {

    // Allocate result
    size_t m = this->size();
    std::vector<Z> z(m);

    // Allocate derivatives
    if (F)
      F->resize(ZDim * m, 6);
    if (E)
      E->resize(ZDim * m, 3);
    if (H && Dim > 6)
      H->resize(ZDim * m, Dim - 6);

    Eigen::Matrix<double, ZDim, 6> Fi;
    Eigen::Matrix<double, ZDim, 3> Ei;
    Eigen::Matrix<double, ZDim, Dim - 6> Hi;

    // Project and fill derivatives
    for (size_t i = 0; i < m; i++) {
      z[i] = this->at(i).project(point, F ? &Fi : 0, E ? &Ei : 0, H ? &Hi : 0);
      if (F)
        F->block<ZDim, 6>(ZDim * i, 0) = Fi;
      if (E)
        E->block<ZDim, 3>(ZDim * i, 0) = Ei;
      if (H)
        H->block<ZDim, Dim - 6>(ZDim * i, 0) = Hi;
    }

    return z;
  }

  /**
   * Project a point, with derivatives in this, point, and calibration
   * throws CheiralityException
   */
  std::vector<Z> projectAtInfinity(const Point3& point) const {

    // Allocate result
    size_t m = this->size();
    std::vector<Z> z(m);

    // Project and fill derivatives
    for (size_t i = 0; i < m; i++)
      z[i] = this->at(i).projectPointAtInfinity(point);

    return z;
  }

  /// Calculate vector of re-projection errors
  Vector reprojectionErrors(const Point3& point,
      const std::vector<Z>& measured) const {
    return ErrorVector(project(point), measured);
  }

  /// Calculate vector of re-projection errors, from point at infinity
  // TODO: take Unit3 instead
  Vector reprojectionErrorsAtInfinity(const Point3& point,
      const std::vector<Z>& measured) const {
    return ErrorVector(projectAtInfinity(point), measured);
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & (*this);
  }
};

template<class CAMERA>
const int CameraSet<CAMERA>::ZDim;

template<class CAMERA>
struct traits<CameraSet<CAMERA> > : public Testable<CameraSet<CAMERA> > {
};

template<class CAMERA>
struct traits<const CameraSet<CAMERA> > : public Testable<CameraSet<CAMERA> > {
};

} // \ namespace gtsam
