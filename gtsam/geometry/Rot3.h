/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot3.h
 * @brief   A common header file for rotation matrix and quaterion rotations, Rot3M and Rot3Q, as well as a typedef of Rot3 to the default implementation.
 * @author  Richard Roberts
 */
// \callgraph

#include <gtsam/geometry/Point3.h>
#include <gtsam/3rdparty/Eigen/Eigen/Geometry>

/* ************************************************************************* */
// Below is the class definition of Rot3.  By the macros at the end of this
// file, both Rot3M and Rot3Q are actually defined with this interface.
#if defined Rot3 || defined __DOXYGEN

namespace gtsam {

  // Forward declarations;
  class Rot3M;
  class Rot3Q;

  /// Typedef to an Eigen Quaternion<double>, we disable alignment because
  /// geometry objects are stored in boost pool allocators, in Values
  /// containers, and and these pool allocators do not support alignment.
  typedef Eigen::Quaternion<double, Eigen::DontAlign> Quaternion;

  /**
   * @brief A 3D rotation represented as a rotation matrix if the preprocessor
   * symbol GTSAM_DEFAULT_QUATERNIONS is not defined, or as a quaternion if it
   * is defined.
   * @ingroup geometry
   * \nosubgrouping
   */
  class Rot3 {
  public:
    static const size_t dimension = 3;

  private:
#if defined ROT3_IS_MATRIX
    /** We store columns ! */
    Point3 r1_, r2_, r3_;
#elif defined ROT3_IS_QUATERNION
    /** Internal Eigen Quaternion */
    Quaternion quaternion_;
#endif

  public:

    /// @name Constructors and named constructors
    /// @{

    /** default constructor, unit rotation */
    Rot3();

    /**
     * Constructor from columns
     * @param r1 X-axis of rotated frame
     * @param r2 Y-axis of rotated frame
     * @param r3 Z-axis of rotated frame
     */
    Rot3(const Point3& r1, const Point3& r2, const Point3& r3);

    /** constructor from a rotation matrix, as doubles in *row-major* order !!! */
    Rot3(double R11, double R12, double R13,
        double R21, double R22, double R23,
        double R31, double R32, double R33);

    /** constructor from a rotation matrix */
    Rot3(const Matrix& R);

    /** Constructor from a quaternion.  This can also be called using a plain
     * Vector, due to implicit conversion from Vector to Quaternion
     * @param q The quaternion
     */
    Rot3(const Quaternion& q);

    /** Constructor from a rotation matrix in a Rot3M */
    Rot3(const Rot3M& r);

    /* Static member function to generate some well known rotations */

    /// Rotation around X axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 Rx(double t);

    /// Rotation around X axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 Ry(double t);

    /// Rotation around X axis as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 Rz(double t);

    /// Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    static Rot3 RzRyRx(double x, double y, double z);

    /// Rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis.
    inline static Rot3 RzRyRx(const Vector& xyz) {
      assert(xyz.size() == 3);
      return RzRyRx(xyz(0), xyz(1), xyz(2));
    }

    /**
     * Positive yaw is to right (as in aircraft heading).
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf.
     * Assumes vehicle coordinate frame X forward, Y right, Z down.
     */
    static Rot3 yaw  (double t) { return Rz(t); }

    /**
     * Positive pitch is up (increasing aircraft altitude).
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf.
     * Assumes vehicle coordinate frame X forward, Y right, Z down.
     */
    static Rot3 pitch(double t) { return Ry(t); }

    /**
     * Positive roll is to right (increasing yaw in aircraft).
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf.
     * Assumes vehicle coordinate frame X forward, Y right, Z down.
     */
    static Rot3 roll (double t) { return Rx(t); }

    /** Returns rotation nRb from body to nav frame.
     * Positive yaw is to right (as in aircraft heading).
     * Positive pitch is up (increasing aircraft altitude).
     * Positive roll is to right (increasing yaw in aircraft).
     * Tait-Bryan system from Spatial Reference Model (SRM) (x,y,z) = (roll,pitch,yaw)
     * as described in http://www.sedris.org/wg8home/Documents/WG80462.pdf.
     * Assumes vehicle coordinate frame X forward, Y right, Z down.
     */
    static Rot3 ypr  (double y, double p, double r) { return RzRyRx(r,p,y);}

    /** Create from Quaternion coefficients */
    static Rot3 quaternion(double w, double x, double y, double z) { Quaternion q(w, x, y, z); return Rot3(q); }

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param   w is the rotation axis, unit length
     * @param   theta rotation angle
     * @return incremental rotation matrix
     */
    static Rot3 rodriguez(const Vector& w, double theta);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param v a vector of incremental roll,pitch,yaw
     * @return incremental rotation matrix
     */
    static Rot3 rodriguez(const Vector& v);

    /**
     * Rodriguez' formula to compute an incremental rotation matrix
     * @param wx Incremental roll (about X)
     * @param wy Incremental pitch (about Y)
     * @param wz Incremental yaw (about Z)
     * @return incremental rotation matrix
     */
    static Rot3 rodriguez(double wx, double wy, double wz)
      { return rodriguez(Vector_(3,wx,wy,wz));}

    /// @}
    /// @name Testable
    /// @{

    /** print */
    void print(const std::string& s="R") const { gtsam::print(matrix(), s);}

    /** equals with an tolerance */
    bool equals(const Rot3& p, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /// identity rotation for group operation
    inline static Rot3 identity() {
      return Rot3();
    }

    /// Compose two rotations i.e., R= (*this) * R2
    Rot3 compose(const Rot3& R2,
    boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /// rotate point from rotated coordinate frame to world = R*p
    Point3 operator*(const Point3& p) const;

    /// derivative of inverse rotation R^T s.t. inverse(R)*R = identity
    Rot3 inverse(boost::optional<Matrix&> H1=boost::none) const;

    /**
     * Return relative rotation D s.t. R2=D*R1, i.e. D=R2*R1'
     */
    Rot3 between(const Rot3& R2,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /** compose two rotations */
    Rot3 operator*(const Rot3& R2) const;

    /**
     * rotate point from rotated coordinate frame to
     * world = R*p
     */
    Point3 rotate(const Point3& p,
    boost::optional<Matrix&> H1=boost::none,  boost::optional<Matrix&> H2=boost::none) const;

    /**
     * rotate point from world to rotated
     * frame = R'*p
     */
    Point3 unrotate(const Point3& p,
      boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const;

    /// @}
    /// @name Manifold
    /// @{

    /// dimension of the variable - used to autodetect sizes
    static size_t Dim() { return dimension; }

    /// return dimensionality of tangent space, DOF = 3
    size_t dim() const { return dimension; }

    /// Retraction from R^3 to Pose2 manifold neighborhood around current pose
    Rot3 retract(const Vector& v) const { return compose(Expmap(v)); }

    /// Returns inverse retraction
    Vector localCoordinates(const Rot3& t2) const { return Logmap(between(t2)); }

    /// @}
    /// @name Lie Group
    /// @{

    /**
     * Exponential map at identity - create a rotation from canonical coordinates
     * using Rodriguez' formula
     */
    static Rot3 Expmap(const Vector& v)  {
      if(zero(v)) return Rot3();
      else return rodriguez(v);
    }

    /**
     * Log map at identity - return the canonical coordinates of this rotation
     */
    static Vector Logmap(const Rot3& R);

    /// @}

    /** return 3*3 rotation matrix */
    Matrix matrix() const;

    /** return 3*3 transpose (inverse) rotation matrix   */
    Matrix transpose() const;

    /** returns column vector specified by index */
    Point3 column(int index) const;
    Point3 r1() const;
    Point3 r2() const;
    Point3 r3() const;

    /**
     * Use RQ to calculate xyz angle representation
     * @return a vector containing x,y,z s.t. R = Rot3::RzRyRx(x,y,z)
     */
    Vector xyz() const;

    /**
     * Use RQ to calculate yaw-pitch-roll angle representation
     * @return a vector containing ypr s.t. R = Rot3::ypr(y,p,r)
     */
    Vector ypr() const;

    /**
     * Use RQ to calculate roll-pitch-yaw angle representation
     * @return a vector containing ypr s.t. R = Rot3::ypr(y,p,r)
     */
    Vector rpy() const;

    /**
     * Accessor to get to component of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double roll() const  { return ypr()(2); }

    /**
     * Accessor to get to component of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double pitch() const { return ypr()(1); }

    /**
     * Accessor to get to component of angle representations
     * NOTE: these are not efficient to get to multiple separate parts,
     * you should instead use xyz() or ypr()
     * TODO: make this more efficient
     */
    inline double yaw() const   { return ypr()(0); }

    /** Compute the quaternion representation of this rotation.
     * @return The quaternion
     */
    Quaternion toQuaternion() const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version)
    {
#if defined ROT3_IS_MATRIX
      ar & BOOST_SERIALIZATION_NVP(r1_);
      ar & BOOST_SERIALIZATION_NVP(r2_);
      ar & BOOST_SERIALIZATION_NVP(r3_);
#elif defined ROT3_IS_QUATERNION
      ar & BOOST_SERIALIZATION_NVP(quaternion_);
#endif
    }
  };

  /**
   * [RQ] receives a 3 by 3 matrix and returns an upper triangular matrix R
   * and 3 rotation angles corresponding to the rotation matrix Q=Qz'*Qy'*Qx'
   * such that A = R*Q = R*Qz'*Qy'*Qx'. When A is a rotation matrix, R will
   * be the identity and Q is a yaw-pitch-roll decomposition of A.
   * The implementation uses Givens rotations and is based on Hartley-Zisserman.
   * @param a 3 by 3 matrix A=RQ
   * @return an upper triangular matrix R
   * @return a vector [thetax, thetay, thetaz] in radians.
   */
  std::pair<Matrix,Vector> RQ(const Matrix& A);
}

#endif // if defined Rot3 || defined __DOXYGEN


/* ************************************************************************* */
// This block of code defines both Rot3Q and Rot3M, by self-including Rot3.h
// twice and using preprocessor definitions of Rot3 to be Rot3M and Rot3Q.  It
// then creates a typedef of Rot3 to either Rot3M or Rot3Q, depending on
// whether GTSAM_DEFAULT_QUATERNIONS is defined.
#if !defined __ROT3_H
#define __ROT3_H

// Define Rot3M
#define Rot3 Rot3M
#define ROT3_IS_MATRIX
#include <gtsam/geometry/Rot3.h>
#undef Rot3
#undef ROT3_IS_MATRIX

// Define Rot3Q
#define Rot3 Rot3Q
#define ROT3_IS_QUATERNION
#include <gtsam/geometry/Rot3.h>
#undef Rot3
#undef ROT3_IS_QUATERNION

// Create Rot3 typedef
namespace gtsam {
  /**
   * Typedef to the main 3D rotation implementation, which is Rot3M by default,
   * or Rot3Q if GTSAM_DEFAULT_QUATERNIONS is defined.  Depending on whether
   * GTSAM_DEFAULT_QUATERNIONS is defined, Rot3M (the rotation matrix
   * implementation) or Rot3Q (the quaternion implementation) will used in all
   * built-in gtsam geometry types that involve 3D rotations, such as Pose3,
   * SimpleCamera, CalibratedCamera, StereoCamera, etc.
   */
#ifdef GTSAM_DEFAULT_QUATERNIONS
  typedef Rot3Q Rot3;
#else
  typedef Rot3M Rot3;
#endif
}

#endif // if !defined Rot3

