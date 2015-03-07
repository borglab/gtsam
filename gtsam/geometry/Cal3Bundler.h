/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Bundler.h
 * @brief Calibration used by Bundler
 * @date Sep 25, 2010
 * @author ydjian
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief Calibration used by Bundler
 * @addtogroup geometry
 * \nosubgrouping
 */
class Cal3Bundler : public DerivedValue<Cal3Bundler> {

private:
	double f_, k1_, k2_ ;

public:

	Matrix K() const ;
	Vector k() const ;

	Vector vector() const;

  /// @name Standard Constructors
  /// @{

	///TODO: comment
	Cal3Bundler() ;

	///TODO: comment
	Cal3Bundler(const double f, const double k1, const double k2) ;

  /// @}
  /// @name Advanced Constructors
  /// @{

	///TODO: comment
	Cal3Bundler(const Vector &v) ;

	/// @}
	/// @name Testable
	/// @{

	/// print with optional string
	void print(const std::string& s = "") const;

	/// assert equality up to a tolerance
	bool equals(const Cal3Bundler& K, double tol = 10e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

	///TODO: comment
	Point2 uncalibrate(const Point2& p,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const ;

	///TODO: comment
	Matrix D2d_intrinsic(const Point2& p) const ;

	///TODO: comment
	Matrix D2d_calibration(const Point2& p) const ;

	///TODO: comment
	Matrix D2d_intrinsic_calibration(const Point2& p) const ;

	/// @}
	/// @name Manifold
	/// @{

	///TODO: comment
	Cal3Bundler retract(const Vector& d) const ;

	///TODO: comment
	Vector localCoordinates(const Cal3Bundler& T2) const ;

	///TODO: comment
	virtual size_t dim() const { return 3 ; }	//TODO: make a final dimension variable (also, usually size_t in other classes  e.g. Pose2)

	///TODO: comment
	static size_t Dim() { return 3; }	//TODO: make a final dimension variable

private:

  /// @}
  /// @name Advanced Interface
  /// @{

	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("Cal3Bundler",
				boost::serialization::base_object<Value>(*this));
		ar & BOOST_SERIALIZATION_NVP(f_);
		ar & BOOST_SERIALIZATION_NVP(k1_);
		ar & BOOST_SERIALIZATION_NVP(k2_);
	}


	/// @}

};

} // namespace gtsam
