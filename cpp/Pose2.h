/**
 * @file  Pose2.h
 * @brief 2D Pose
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

// \callgraph

#pragma once

#include <boost/optional.hpp>
#include "Matrix.h"
#include "Testable.h"
#include "Lie.h"
#include "Point2.h"
#include "Rot2.h"

namespace gtsam {

  /**
   * A 2D pose (Point2,Rot2)
   */
  class Pose2: Testable<Pose2>, public Lie<Pose2>  {
  private:
    Point2 t_;
    Rot2 r_;

  public:

    /** default constructor = origin */
    Pose2() {} // default is origin

    /** copy constructor */
    Pose2(const Pose2& pose) : t_(pose.t_), r_(pose.r_) {}

    /**
     * construct from (x,y,theta)
     * @param x x coordinate
     * @param y y coordinate
     * @param theta angle with positive X-axis
     */
    Pose2(double x, double y, double theta) :
			t_(x, y), r_(Rot2::fromAngle(theta)) {
		}

    /** construct from rotation and translation */
    Pose2(double theta, const Point2& t) :
			t_(t), r_(Rot2::fromAngle(theta)) {
		}
    Pose2(const Rot2& r, const Point2& t) : t_(t), r_(r) {}

    /** Constructor from 3*3 matrix */
    Pose2(const Matrix &T) :
      r_(Rot2::atan2(T(1, 0), T(0, 0))), t_(T(0, 2), T(1, 2)) {}

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** assert equality up to a tolerance */
    bool equals(const Pose2& pose, double tol = 1e-9) const;

    /** return transformation matrix */
    Matrix matrix() const;

    /** get functions for x, y, theta */
    inline double x()     const { return t_.x(); }
    inline double y()     const { return t_.y(); }
    inline double theta() const { return r_.theta(); }

    inline const Point2& t() const { return t_; }
    inline const Rot2&   r() const { return r_; }

    static inline size_t dim() { return 3; };

  private:
    // Serialization function
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(t_);
		ar & BOOST_SERIALIZATION_NVP(r_);
    }
  }; // Pose2

  /** print using member print function, currently used by LieConfig */
  inline void print(const Pose2& obj, const std::string& str = "") { obj.print(str); }

  /** return DOF, dimensionality of tangent space = 3 */
  inline size_t dim(const Pose2&) { return 3; }

  /**
   * Exponential map from se(2) to SE(2)
   */
  template<> Pose2 expmap(const Vector& v);

  /**
   * Inverse of exponential map, from SE(2) to se(2)
   */
  Vector logmap(const Pose2& p);

  /**
  * Calculate Adjoint map
  * Ad_pose is 3*3 matrix that when applied to twist xi, returns Ad_pose(xi)
  */
  Matrix AdjointMap(const Pose2& p);
  inline Vector Adjoint(const Pose2& p, const Vector& xi) { return AdjointMap(p)*xi;}

  /**
   * wedge for SE(2):
   * @param xi 3-dim twist (v,omega) where
   *  omega is angular velocity
   *  v (vx,vy) = 2D velocity
   * @return xihat, 3*3 element of Lie algebra that can be exponentiated
   */
  inline Matrix wedge(double vx, double vy, double w) {
  	return Matrix_(3,3,
  			 0.,-w,  vx,
  			 w,  0., vy,
  			 0., 0.,  0.);
  }

  template <>
  inline Matrix wedge<Pose2>(const Vector& xi) {
  	return wedge(xi(0),xi(1),xi(2));
  }

  /**
   * inverse transformation
   */
  Pose2 inverse(const Pose2& pose);
  Matrix Dinverse(const Pose2& pose);

  /**
   * compose this transformation onto another (first p1 and then p2)
   */
  inline Pose2 compose(const Pose2& p0, const Pose2& p1)
		{ return Pose2(p0.r()*p1.r(), p0.t() + p0.r()*p1.t());}
  Matrix Dcompose1(const Pose2& p1, const Pose2& p2);
  Matrix Dcompose2(const Pose2& p1, const Pose2& p2);
  inline Pose2 operator*(const Pose2& p1, const Pose2& p0) { return compose(p1, p0);}

  /**
   * Return point coordinates in pose coordinate frame
   */
  inline Point2 transform_to(const Pose2& pose, const Point2& point)
		{ return unrotate(pose.r(), point - pose.t());}
  Point2 transform_to(const Pose2& pose, const Point2& point,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2);

  /**
   * Return point coordinates in global frame
   */
  inline Point2 transform_from(const Pose2& pose, const Point2& point)
		{ return rotate(pose.r(), point) + pose.t();}
  Point2 transform_from(const Pose2& pose, const Point2& point,
  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2);
  inline Point2 operator*(const Pose2& pose, const Point2& point)
		{ return transform_from(pose, point);}

  /**
   * Return relative pose between p1 and p2, in p1 coordinate frame
   */
  Pose2 between(const Pose2& p1, const Pose2& p2,
  	boost::optional<Matrix&> H1, boost::optional<Matrix&> H2);

	/**
	 * Calculate bearing to a landmark
	 * @param pose 2D pose of robot
	 * @param point 2D location of landmark
	 * @return 2D rotation \in SO(2)
	 */
	Rot2 bearing(const Pose2& pose, const Point2& point);
	Rot2 bearing(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2);

	/**
	 * Calculate range to a landmark
	 * @param pose 2D pose of robot
	 * @param point 2D location of landmark
	 * @return range (double)
	 */
	double range(const Pose2& pose, const Point2& point);
	double range(const Pose2& pose, const Point2& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2);

} // namespace gtsam

