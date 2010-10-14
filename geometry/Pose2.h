/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Pose2.h
 * @brief 2D Pose
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

// \callgraph

#pragma once

#include <boost/optional.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>

namespace gtsam {

  /**
   * A 2D pose (Point2,Rot2)
   */
  class Pose2: Testable<Pose2>, public Lie<Pose2>  {

  public:
	  static const size_t dimension = 3;
  private:
    Rot2 r_;
    Point2 t_;

  public:

    /** default constructor = origin */
    Pose2() {} // default is origin

    /** copy constructor */
    Pose2(const Pose2& pose) : r_(pose.r_), t_(pose.t_) {}

    /**
     * construct from (x,y,theta)
     * @param x x coordinate
     * @param y y coordinate
     * @param theta angle with positive X-axis
     */
    Pose2(double x, double y, double theta) :
    	r_(Rot2::fromAngle(theta)), t_(x, y) {
		}

    /** construct from rotation and translation */
    Pose2(double theta, const Point2& t) :
    	r_(Rot2::fromAngle(theta)), t_(t) {
		}
    Pose2(const Rot2& r, const Point2& t) : r_(r), t_(t) {}

    /** Constructor from 3*3 matrix */
    Pose2(const Matrix &T) :
      r_(Rot2::atan2(T(1, 0), T(0, 0))), t_(T(0, 2), T(1, 2)) { assert(T.size1()==3 && T.size2()==3); }

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** assert equality up to a tolerance */
    bool equals(const Pose2& pose, double tol = 1e-9) const;

    /** compose syntactic sugar */
    inline Pose2 operator*(const Pose2& p2) const {
    	return Pose2(r_*p2.r(), t_ + r_*p2.t());
    }

    /** dimension of the variable - used to autodetect sizes */
    inline static size_t Dim() { return dimension; }

    /** Lie requirements */

    /** return DOF, dimensionality of tangent space = 3 */
    inline size_t dim() const { return dimension; }

    /**
     * inverse transformation with derivatives
     */
    Pose2 inverse(boost::optional<Matrix&> H1=boost::none) const;

    /**
     * compose this transformation onto another (first *this and then p2)
     * With optional derivatives
     */
    Pose2 compose(const Pose2& p2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const;

    /** syntactic sugar for transform_from */
    inline Point2 operator*(const Point2& point) { return transform_from(point);}

    /**
     * Exponential map from se(2) to SE(2)
     */
    static Pose2 Expmap(const Vector& v);

    /**
     * Inverse of exponential map, from SE(2) to se(2)
     */
    static Vector Logmap(const Pose2& p);

    /** default implementations of binary functions */
    inline Pose2 expmap(const Vector& v) const { return gtsam::expmap_default(*this, v); }
    inline Vector logmap(const Pose2& p2) const { return gtsam::logmap_default(*this, p2);}

    /**
     * Return relative pose between p1 and p2, in p1 coordinate frame
     */
    Pose2 between(const Pose2& p2,
    	boost::optional<Matrix&> H1=boost::none,
    	boost::optional<Matrix&> H2=boost::none) const;

    /** return transformation matrix */
    Matrix matrix() const;

    /**
     * Return point coordinates in pose coordinate frame
     */
    Point2 transform_to(const Point2& point,
    		boost::optional<Matrix&> H1=boost::none,
    		boost::optional<Matrix&> H2=boost::none) const;

    /**
     * Return point coordinates in global frame
     */
    Point2 transform_from(const Point2& point,
    	boost::optional<Matrix&> H1=boost::none,
    	boost::optional<Matrix&> H2=boost::none) const;

	/**
	 * Calculate bearing to a landmark
	 * @param point 2D location of landmark
	 * @return 2D rotation \in SO(2)
	 */
	Rot2 bearing(const Point2& point,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const;

	/**
	 * Calculate range to a landmark
	 * @param point 2D location of landmark
	 * @return range (double)
	 */
	double range(const Point2& point,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const;

	/**
	 * Calculate Adjoint map
	 * Ad_pose is 3*3 matrix that when applied to twist xi, returns Ad_pose(xi)
	 */
	Matrix AdjointMap() const;
	inline Vector Adjoint(const Vector& xi) const {
	  assert(xi.size() == 3);
		return AdjointMap()*xi;
	}

	/**
	 * wedge for SE(2):
	 * @param xi 3-dim twist (v,omega) where
	 *  omega is angular velocity
	 *  v (vx,vy) = 2D velocity
	 * @return xihat, 3*3 element of Lie algebra that can be exponentiated
	 */
	static inline Matrix wedge(double vx, double vy, double w) {
		return Matrix_(3,3,
				0.,-w,  vx,
				w,  0., vy,
				0., 0.,  0.);
	}

    /** get functions for x, y, theta */
    inline double x()     const { return t_.x(); }
    inline double y()     const { return t_.y(); }
    inline double theta() const { return r_.theta(); }

    inline const Point2& t() const { return t_; }
    inline const Rot2&   r() const { return r_; }

  private:
    // Serialization function
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(t_);
		ar & BOOST_SERIALIZATION_NVP(r_);
    }
  }; // Pose2

  /** specialization for pose2 wedge function (generic template in Lie.h) */
  template <>
  inline Matrix wedge<Pose2>(const Vector& xi) {
  	return Pose2::wedge(xi(0),xi(1),xi(2));
  }

  /**
   * Calculate pose between a vector of 2D point correspondences (p,q)
   * where q = Pose2::transform_from(p) = t + R*p
   */
  typedef std::pair<Point2,Point2> Point2Pair;
  boost::optional<Pose2> align(const std::vector<Point2Pair>& pairs);

} // namespace gtsam

