/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieMatrix.h
 * @brief A wrapper around Matrix providing Lie compatibility
 * @author Richard Roberts and Alex Cunningham
 */

#pragma once

#include <cstdarg>

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/DerivedValue.h>
#include <boost/serialization/nvp.hpp>

namespace gtsam {

/**
 * LieVector is a wrapper around vector to allow it to be a Lie type
 */
struct LieMatrix : public Matrix, public DerivedValue<LieMatrix> {

	/** default constructor - should be unnecessary */
	LieMatrix() {}

	/** initialize from a normal matrix */
	LieMatrix(const Matrix& v) : Matrix(v) {}

	/** initialize from a fixed size normal vector */
	template<int M, int N>
	LieMatrix(const Eigen::Matrix<double, M, N>& v) : Matrix(v) {}

	/** constructor with size and initial data, row order ! */
	LieMatrix(size_t m, size_t n, const double* const data) :
      Matrix(Eigen::Map<const Matrix>(data, m, n)) {}

	/** Specify arguments directly, as in Matrix_() - always force these to be doubles */
	LieMatrix(size_t m, size_t n, ...);

	/** get the underlying vector */
	inline Matrix matrix() const {
		return static_cast<Matrix>(*this);
	}

	/** print @param s optional string naming the object */
	inline void print(const std::string& name="") const {
		gtsam::print(matrix(), name);
	}

	/** equality up to tolerance */
	inline bool equals(const LieMatrix& expected, double tol=1e-5) const {
		return gtsam::equal_with_abs_tol(matrix(), expected.matrix(), tol);
	}

	// Manifold requirements

	/** Returns dimensionality of the tangent space */
	inline size_t dim() const { return this->size(); }

	/** Update the LieMatrix with a tangent space update.  The elements of the
   * tangent space vector correspond to the matrix entries arranged in
   * *row-major* order. */
	inline LieMatrix retract(const Vector& v) const {
    if(v.size() != this->rows() * this->cols())
      throw std::invalid_argument("LieMatrix::retract called with Vector of incorrect size");
    return LieMatrix(*this + Eigen::Map<const Matrix>(&v(0), this->cols(), this->rows()).transpose());
  }

	/** @return the local coordinates of another object.  The elements of the
   * tangent space vector correspond to the matrix entries arranged in
   * *row-major* order. */
	inline Vector localCoordinates(const LieMatrix& t2) const {
    Vector result(this->rows() * this->cols());
    Eigen::Map<Matrix>(&result(0), this->cols(), this->rows()).transpose() =
      (t2 - *this);
    return result;
  }

	// Group requirements

	/** identity - NOTE: no known size at compile time - so zero length */
	inline static LieMatrix identity() {
		throw std::runtime_error("LieMatrix::identity(): Don't use this function");
		return LieMatrix();
	}

	// Note: Manually specifying the 'gtsam' namespace for the optional Matrix arguments
	// This is a work-around for linux g++ 4.6.1 that incorrectly selects the Eigen::Matrix class
	// instead of the gtsam::Matrix class. This is related to deriving this class from an Eigen Vector
	// as the other geometry objects (Point3, Rot3, etc.) have this problem
	/** compose with another object */
	inline LieMatrix compose(const LieMatrix& p,
			boost::optional<gtsam::Matrix&> H1=boost::none,
			boost::optional<gtsam::Matrix&> H2=boost::none) const {
		if(H1) *H1 = eye(dim());
		if(H2) *H2 = eye(p.dim());

		return LieMatrix(*this + p);
	}

	/** between operation */
	inline LieMatrix between(const LieMatrix& l2,
			boost::optional<gtsam::Matrix&> H1=boost::none,
			boost::optional<gtsam::Matrix&> H2=boost::none) const {
		if(H1) *H1 = -eye(dim());
		if(H2) *H2 = eye(l2.dim());
		return LieMatrix(l2 - *this);
	}

	/** invert the object and yield a new one */
	inline LieMatrix inverse(boost::optional<gtsam::Matrix&> H=boost::none) const {
		if(H) *H = -eye(dim());

		return LieMatrix(-(*this));
	}

	// Lie functions

	/** Expmap around identity */
	static inline LieMatrix Expmap(const Vector& v) {
    throw std::runtime_error("LieMatrix::Expmap(): Don't use this function");
    return LieMatrix(v); }

	/** Logmap around identity - just returns with default cast back */
	static inline Vector Logmap(const LieMatrix& p) {
    return Eigen::Map<const Vector>(&p(0,0), p.dim()); }

private:

  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("LieMatrix",
       boost::serialization::base_object<Value>(*this));
    ar & boost::serialization::make_nvp("Matrix",
       boost::serialization::base_object<Matrix>(*this));

  }

};
} // \namespace gtsam
