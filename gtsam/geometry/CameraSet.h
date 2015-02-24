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
  typedef std::vector<MatrixZD> FBlocks;

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
   * Project a point, with derivatives in CameraSet and Point3
   * Note that F is a sparse block-diagonal matrix, so instead of a large dense
   * matrix this function returns the diagonal blocks.
   * throws CheiralityException
   */
  std::vector<Z> project2(const Point3& point, //
      boost::optional<FBlocks&> F = boost::none, //
      boost::optional<Matrix&> E = boost::none) const {

    // Allocate result
    size_t m = this->size();
    std::vector<Z> z(m);

    // Allocate derivatives
    if (E)
      E->resize(ZDim * m, 3);

    // Project and fill derivatives
    for (size_t i = 0; i < m; i++) {
      Eigen::Matrix<double, ZDim, Dim> Fi;
      Eigen::Matrix<double, ZDim, 3> Ei;
      z[i] = this->at(i).project2(point, F ? &Fi : 0, E ? &Ei : 0);
      if (F)
        F->push_back(Fi);
      if (E)
        E->block<ZDim, 3>(ZDim * i, 0) = Ei;
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
  Vector reprojectionError(const Point3& point, const std::vector<Z>& measured,
      boost::optional<FBlocks&> F = boost::none, //
      boost::optional<Matrix&> E = boost::none) const {
    return ErrorVector(project2(point,F,E), measured);
  }

  /// Calculate vector of re-projection errors, from point at infinity
  // TODO: take Unit3 instead
  Vector reprojectionErrorAtInfinity(const Point3& point,
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
