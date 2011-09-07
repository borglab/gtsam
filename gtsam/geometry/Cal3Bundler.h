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

#include <gtsam/geometry/Point2.h>

namespace gtsam {

/**
 * @brief Calibration used by Bundler
 * @ingroup geometry
 */
class Cal3Bundler: public Testable<Cal3Bundler> {

private:
	double f_, k1_, k2_ ;

public:

	Cal3Bundler() ;
	Cal3Bundler(const Vector &v) ;
	Cal3Bundler(const double f, const double k1, const double k2) ;
	Matrix K() const ;
	Vector k() const ;

	Vector vector() const;
	void print(const std::string& s = "") const;
	bool equals(const Cal3Bundler& K, double tol = 10e-9) const;

	Point2 uncalibrate(const Point2& p,
					   boost::optional<Matrix&> H1 = boost::none,
					   boost::optional<Matrix&> H2 = boost::none) const ;

	Matrix D2d_intrinsic(const Point2& p) const ;
	Matrix D2d_calibration(const Point2& p) const ;
	Matrix D2d_intrinsic_calibration(const Point2& p) const ;

	Cal3Bundler expmap(const Vector& d) const ;
	Vector logmap(const Cal3Bundler& T2) const ;

    static Cal3Bundler Expmap(const Vector& v) ;
	static Vector Logmap(const Cal3Bundler& p) ;

	int dim() const { return 3 ; }
	static size_t Dim() { return 3; }

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & BOOST_SERIALIZATION_NVP(f_);
		ar & BOOST_SERIALIZATION_NVP(k1_);
		ar & BOOST_SERIALIZATION_NVP(k2_);
	}

};

} // namespace gtsam
