/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearizedFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <vector>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * A dummy factor that takes a linearized factor and inserts it into
 * a nonlinear graph.
 */
class LinearizedFactor : public gtsam::NoiseModelFactor {

public:
	/** base type */
	typedef gtsam::NoiseModelFactor Base;
	typedef LinearizedFactor This;

	/** shared pointer for convenience */
	typedef boost::shared_ptr<LinearizedFactor > shared_ptr;


	/** decoder for keys - avoids the use of a full ordering */
	typedef gtsam::Ordering::InvertedMap KeyLookup;

protected:
	/** hold onto the factor itself */
	// store components of a jacobian factor
	typedef std::map<Key, gtsam::Matrix> KeyMatrixMap;
	KeyMatrixMap matrices_;
	gtsam::Vector b_;
	gtsam::SharedDiagonal model_; /// separate from the noisemodel in NonlinearFactor due to Diagonal/Gaussian

	/** linearization points for error calculation */
	gtsam::Values lin_points_;

	/** default constructor for serialization */
	LinearizedFactor() {}

public:

	virtual ~LinearizedFactor() {}

	/// @return a deep copy of this factor
	virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

	/**
	 * Use this constructor with pre-constructed decoders
	 * @param lin_factor is a gaussian factor with linear keys (no labels baked in)
	 * @param decoder is a structure to look up the correct keys
	 * @param values is assumed to have the correct key structure with labels
	 */
	LinearizedFactor(gtsam::JacobianFactor::shared_ptr lin_factor,
			const KeyLookup& decoder, const gtsam::Values& lin_points);

	/**
	 * Use this constructor with the ordering used to linearize the graph
	 * @param lin_factor is a gaussian factor with linear keys (no labels baked in)
	 * @param ordering is the full ordering used to linearize the graph
	 * @param values is assumed to have the correct key structure with labels
	 */
	LinearizedFactor(gtsam::JacobianFactor::shared_ptr lin_factor,
			const gtsam::Ordering& ordering, const gtsam::Values& lin_points);

	/** Vector of errors, unwhitened, with optional derivatives (ordered by key) */
	virtual gtsam::Vector unwhitenedError(const gtsam::Values& c,
			boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const;

	/**
	 * linearize to a GaussianFactor
	 * Reimplemented from NoiseModelFactor to directly copy out the
	 * matrices and only update the RHS b with an updated residual
	 */
	virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(
			const gtsam::Values& c, const gtsam::Ordering& ordering) const;

	// Testable

	/** print function */
	virtual void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

	/** equals function with optional tolerance */
	virtual bool equals(const NonlinearFactor& other, double tol = 1e-9) const;

	// access functions

	const gtsam::Vector& b() const { return b_; }
	inline const gtsam::Values& linearizationPoint() const { return lin_points_; }

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class ARCHIVE>
	void serialize(ARCHIVE & ar, const unsigned int version) {
		ar & boost::serialization::make_nvp("NonlinearFactor",
				boost::serialization::base_object<Base>(*this));
		ar & BOOST_SERIALIZATION_NVP(matrices_);
		ar & BOOST_SERIALIZATION_NVP(b_);
		ar & BOOST_SERIALIZATION_NVP(model_);
		ar & BOOST_SERIALIZATION_NVP(lin_points_);
	}
};

} // \namespace gtsam

