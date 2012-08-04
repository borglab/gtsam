/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieVector.h
 * @brief A wrapper around vector providing Lie compatibility
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/DerivedValue.h>

namespace gtsam {

/**
 * LieVector is a wrapper around vector to allow it to be a Lie type
 */
struct LieVector : public Vector, public DerivedValue<LieVector> {

	/** default constructor - should be unnecessary */
	LieVector() {}

	/** initialize from a normal vector */
	LieVector(const Vector& v) : Vector(v) {}

	/** initialize from a fixed size normal vector */
	template<int N>
	LieVector(const Eigen::Matrix<double, N, 1>& v) : Vector(v) {}

	/** wrap a double */
	LieVector(double d) : Vector(Vector_(1, d)) {}

	/** constructor with size and initial data, row order ! */
	LieVector(size_t m, const double* const data);

	/** Specify arguments directly, as in Vector_() - always force these to be doubles */
	LieVector(size_t m, ...);

	/** get the underlying vector */
	inline Vector vector() const {
		return static_cast<Vector>(*this);
	}

	/** print @param name optional string naming the object */
	inline void print(const std::string& name="") const {
		gtsam::print(vector(), name);
	}

	/** equality up to tolerance */
	inline bool equals(const LieVector& expected, double tol=1e-5) const {
		return gtsam::equal(vector(), expected.vector(), tol);
	}

	// Manifold requirements

	/** Returns dimensionality of the tangent space */
	inline size_t dim() const { return this->size(); }

	/** Update the LieVector with a tangent space update */
	inline LieVector retract(const Vector& v) const { return LieVector(vector() + v); }

	/** @return the local coordinates of another object */
	inline Vector localCoordinates(const LieVector& t2) const { return LieVector(t2 - vector()); }

	// Group requirements

	/** identity - NOTE: no known size at compile time - so zero length */
	inline static LieVector identity() {
		throw std::runtime_error("LieVector::identity(): Don't use this function");
		return LieVector();
	}

	// Note: Manually specifying the 'gtsam' namespace for the optional Matrix arguments
	// This is a work-around for linux g++ 4.6.1 that incorrectly selects the Eigen::Matrix class
	// instead of the gtsam::Matrix class. This is related to deriving this class from an Eigen Vector
	// as the other geometry objects (Point3, Rot3, etc.) have this problem
	/** compose with another object */
	inline LieVector compose(const LieVector& p,
			boost::optional<gtsam::Matrix&> H1=boost::none,
			boost::optional<gtsam::Matrix&> H2=boost::none) const {
		if(H1) *H1 = eye(dim());
		if(H2) *H2 = eye(p.dim());

		return LieVector(vector() + p);
	}

	/** between operation */
	inline LieVector between(const LieVector& l2,
			boost::optional<gtsam::Matrix&> H1=boost::none,
			boost::optional<gtsam::Matrix&> H2=boost::none) const {
		if(H1) *H1 = -eye(dim());
		if(H2) *H2 = eye(l2.dim());
		return LieVector(l2.vector() - vector());
	}

	/** invert the object and yield a new one */
	inline LieVector inverse(boost::optional<gtsam::Matrix&> H=boost::none) const {
		if(H) *H = -eye(dim());

		return LieVector(-1.0 * vector());
	}

	// Lie functions

	/** Expmap around identity */
	static inline LieVector Expmap(const Vector& v) { return LieVector(v); }

	/** Logmap around identity - just returns with default cast back */
	static inline Vector Logmap(const LieVector& p) { return p; }

private:

  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("LieVector",
       boost::serialization::base_object<Value>(*this));
    ar & boost::serialization::make_nvp("Vector",
       boost::serialization::base_object<Vector>(*this));
  }

};
} // \namespace gtsam
