/**
 * \file GravityModel.hpp
 * \brief Header for GeographicLib::GravityModel class
 *
 * Copyright (c) Charles Karney (2011-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GRAVITYMODEL_HPP)
#define GEOGRAPHICLIB_GRAVITYMODEL_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/NormalGravity.hpp>
#include <GeographicLib/SphericalHarmonic.hpp>
#include <GeographicLib/SphericalHarmonic1.hpp>

#if defined(_MSC_VER)
// Squelch warnings about dll vs vector
#  pragma warning (push)
#  pragma warning (disable: 4251)
#endif

namespace GeographicLib {

  class GravityCircle;

  /**
   * \brief Model of the earth's gravity field
   *
   * Evaluate the earth's gravity field according to a model.  The supported
   * models treat only the gravitational field exterior to the mass of the
   * earth.  When computing the field at points near (but above) the surface of
   * the earth a small correction can be applied to account for the mass of the
   * atmosphere above the point in question; see \ref gravityatmos.
   * Determining the height of the geoid above the ellipsoid entails correcting
   * for the mass of the earth above the geoid.  The egm96 and egm2008 include
   * separate correction terms to account for this mass.
   *
   * Definitions and terminology (from Heiskanen and Moritz, Sec 2-13):
   * - \e V = gravitational potential;
   * - &Phi; = rotational potential;
   * - \e W = \e V + &Phi; = \e T + \e U = total potential;
   * - <i>V</i><sub>0</sub> = normal gravitation potential;
   * - \e U = <i>V</i><sub>0</sub> + &Phi; = total normal potential;
   * - \e T = \e W &minus; \e U = \e V &minus; <i>V</i><sub>0</sub> = anomalous
   *   or disturbing potential;
   * - <b>g</b> = &nabla;\e W = <b>&gamma;</b> + <b>&delta;</b>;
   * - <b>f</b> = &nabla;&Phi;;
   * - <b>&Gamma;</b> = &nabla;<i>V</i><sub>0</sub>;
   * - <b>&gamma;</b> = &nabla;\e U;
   * - <b>&delta;</b> = &nabla;\e T = gravity disturbance vector
   *   = <b>g</b><sub><i>P</i></sub> &minus; <b>&gamma;</b><sub><i>P</i></sub>;
   * - &delta;\e g = gravity disturbance = <i>g</i><sub><i>P</i></sub> &minus;
   *   &gamma;<sub><i>P</i></sub>;
   * - &Delta;<b>g</b> = gravity anomaly vector = <b>g</b><sub><i>P</i></sub>
   *   &minus; <b>&gamma;</b><sub><i>Q</i></sub>; here the line \e PQ is
   *   perpendicular to ellipsoid and the potential at \e P equals the normal
   *   potential at \e Q;
   * - &Delta;\e g = gravity anomaly = <i>g</i><sub><i>P</i></sub> &minus;
   *   &gamma;<sub><i>Q</i></sub>;
   * - (&xi;, &eta;) deflection of the vertical, the difference in
   *   directions of <b>g</b><sub><i>P</i></sub> and
   *   <b>&gamma;</b><sub><i>Q</i></sub>, &xi; = NS, &eta; = EW.
   * - \e X, \e Y, \e Z, geocentric coordinates;
   * - \e x, \e y, \e z, local cartesian coordinates used to denote the east,
   *   north and up directions.
   *
   * See \ref gravity for details of how to install the gravity models and the
   * data format.
   *
   * References:
   * - W. A. Heiskanen and H. Moritz, Physical Geodesy (Freeman, San
   *   Francisco, 1967).
   *
   * Example of use:
   * \include example-GravityModel.cpp
   *
   * <a href="Gravity.1.html">Gravity</a> is a command-line utility providing
   * access to the functionality of GravityModel and GravityCircle.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT GravityModel {
  private:
    typedef Math::real real;
    friend class GravityCircle;
    static const int idlength_ = 8;
    std::string _name, _dir, _description, _date, _filename, _id;
    real _amodel, _GMmodel, _zeta0, _corrmult;
    SphericalHarmonic::normalization _norm;
    NormalGravity _earth;
    std::vector<real> _Cx, _Sx, _CC, _CS, _zonal;
    real _dzonal0;              // A left over contribution to _zonal.
    SphericalHarmonic _gravitational;
    SphericalHarmonic1 _disturbing;
    SphericalHarmonic _correction;
    void ReadMetadata(const std::string& name);
    Math::real InternalT(real X, real Y, real Z,
                         real& deltaX, real& deltaY, real& deltaZ,
                         bool gradp, bool correct) const;
    GravityModel(const GravityModel&); // copy constructor not allowed
    GravityModel& operator=(const GravityModel&); // nor copy assignment

    enum captype {
      CAP_NONE   = 0U,
      CAP_G      = 1U<<0,       // implies potentials W and V
      CAP_T      = 1U<<1,
      CAP_DELTA  = 1U<<2 | CAP_T, // delta implies T?
      CAP_C      = 1U<<3,
      CAP_GAMMA0 = 1U<<4,
      CAP_GAMMA  = 1U<<5,
      CAP_ALL    = 0x3FU,
    };

  public:

    /**
     * Bit masks for the capabilities to be given to the GravityCircle object
     * produced by Circle.
     **********************************************************************/
    enum mask {
      /**
       * No capabilities.
       * @hideinitializer
       **********************************************************************/
      NONE = 0U,
      /**
       * Allow calls to GravityCircle::Gravity, GravityCircle::W, and
       * GravityCircle::V.
       * @hideinitializer
       **********************************************************************/
      GRAVITY = CAP_G,
      /**
       * Allow calls to GravityCircle::Disturbance and GravityCircle::T.
       * @hideinitializer
       **********************************************************************/
      DISTURBANCE = CAP_DELTA,
      /**
       * Allow calls to GravityCircle::T(real lon) (i.e., computing the
       * disturbing potential and not the gravity disturbance vector).
       * @hideinitializer
       **********************************************************************/
      DISTURBING_POTENTIAL = CAP_T,
      /**
       * Allow calls to GravityCircle::SphericalAnomaly.
       * @hideinitializer
       **********************************************************************/
      SPHERICAL_ANOMALY = CAP_DELTA | CAP_GAMMA,
      /**
       * Allow calls to GravityCircle::GeoidHeight.
       * @hideinitializer
       **********************************************************************/
      GEOID_HEIGHT = CAP_T | CAP_C | CAP_GAMMA0,
      /**
       * All capabilities.
       * @hideinitializer
       **********************************************************************/
      ALL = CAP_ALL,
    };
    /** \name Setting up the gravity model
     **********************************************************************/
    ///@{
    /**
     * Construct a gravity model.
     *
     * @param[in] name the name of the model.
     * @param[in] path (optional) directory for data file.
     * @exception GeographicErr if the data file cannot be found, is
     *   unreadable, or is corrupt.
     * @exception std::bad_alloc if the memory necessary for storing the model
     *   can't be allocated.
     *
     * A filename is formed by appending ".egm" (World Gravity Model) to the
     * name.  If \e path is specified (and is non-empty), then the file is
     * loaded from directory, \e path.  Otherwise the path is given by
     * DefaultGravityPath().
     *
     * This file contains the metadata which specifies the properties of the
     * model.  The coefficients for the spherical harmonic sums are obtained
     * from a file obtained by appending ".cof" to metadata file (so the
     * filename ends in ".egm.cof").
     **********************************************************************/
    explicit GravityModel(const std::string& name,
                          const std::string& path = "");
    ///@}

    /** \name Compute gravity in geodetic coordinates
     **********************************************************************/
    ///@{
    /**
     * Evaluate the gravity at an arbitrary point above (or below) the
     * ellipsoid.
     *
     * @param[in] lat the geographic latitude (degrees).
     * @param[in] lon the geographic longitude (degrees).
     * @param[in] h the height above the ellipsoid (meters).
     * @param[out] gx the easterly component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gy the northerly component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gz the upward component of the acceleration
     *   (m s<sup>&minus;2</sup>); this is usually negative.
     * @return \e W the sum of the gravitational and centrifugal potentials
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     *
     * The function includes the effects of the earth's rotation.
     **********************************************************************/
    Math::real Gravity(real lat, real lon, real h,
                       real& gx, real& gy, real& gz) const;

    /**
     * Evaluate the gravity disturbance vector at an arbitrary point above (or
     * below) the ellipsoid.
     *
     * @param[in] lat the geographic latitude (degrees).
     * @param[in] lon the geographic longitude (degrees).
     * @param[in] h the height above the ellipsoid (meters).
     * @param[out] deltax the easterly component of the disturbance vector
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltay the northerly component of the disturbance vector
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltaz the upward component of the disturbance vector
     *   (m s<sup>&minus;2</sup>).
     * @return \e T the corresponding disturbing potential
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real Disturbance(real lat, real lon, real h,
                           real& deltax, real& deltay, real& deltaz)
      const;

    /**
     * Evaluate the geoid height.
     *
     * @param[in] lat the geographic latitude (degrees).
     * @param[in] lon the geographic longitude (degrees).
     * @return \e N the height of the geoid above the ReferenceEllipsoid()
     *   (meters).
     *
     * This calls NormalGravity::U for ReferenceEllipsoid().  Some
     * approximations are made in computing the geoid height so that the
     * results of the NGA codes are reproduced accurately.  Details are given
     * in \ref gravitygeoid.
     **********************************************************************/
    Math::real GeoidHeight(real lat, real lon) const;

    /**
     * Evaluate the components of the gravity anomaly vector using the
     * spherical approximation.
     *
     * @param[in] lat the geographic latitude (degrees).
     * @param[in] lon the geographic longitude (degrees).
     * @param[in] h the height above the ellipsoid (meters).
     * @param[out] Dg01 the gravity anomaly (m s<sup>&minus;2</sup>).
     * @param[out] xi the northerly component of the deflection of the vertical
     *  (degrees).
     * @param[out] eta the easterly component of the deflection of the vertical
     *  (degrees).
     *
     * The spherical approximation (see Heiskanen and Moritz, Sec 2-14) is used
     * so that the results of the NGA codes are reproduced accurately.
     * approximations used here.  Details are given in \ref gravitygeoid.
     **********************************************************************/
    void SphericalAnomaly(real lat, real lon, real h,
                          real& Dg01, real& xi, real& eta) const;
    ///@}

    /** \name Compute gravity in geocentric coordinates
     **********************************************************************/
    ///@{
    /**
     * Evaluate the components of the acceleration due to gravity and the
     * centrifugal acceleration in geocentric coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[in] Z geocentric coordinate of point (meters).
     * @param[out] gX the \e X component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gY the \e Y component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gZ the \e Z component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @return \e W = \e V + &Phi; the sum of the gravitational and
     *   centrifugal potentials (m<sup>2</sup> s<sup>&minus;2</sup>).
     *
     * This calls NormalGravity::U for ReferenceEllipsoid().
     **********************************************************************/
    Math::real W(real X, real Y, real Z,
                 real& gX, real& gY, real& gZ) const;

    /**
     * Evaluate the components of the acceleration due to gravity in geocentric
     * coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[in] Z geocentric coordinate of point (meters).
     * @param[out] GX the \e X component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] GY the \e Y component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] GZ the \e Z component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @return \e V = \e W - &Phi; the gravitational potential
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real V(real X, real Y, real Z,
                 real& GX, real& GY, real& GZ) const;

    /**
     * Evaluate the components of the gravity disturbance in geocentric
     * coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[in] Z geocentric coordinate of point (meters).
     * @param[out] deltaX the \e X component of the gravity disturbance
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltaY the \e Y component of the gravity disturbance
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltaZ the \e Z component of the gravity disturbance
     *   (m s<sup>&minus;2</sup>).
     * @return \e T = \e W - \e U the disturbing potential (also called the
     *   anomalous potential) (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real T(real X, real Y, real Z,
                 real& deltaX, real& deltaY, real& deltaZ) const
    { return InternalT(X, Y, Z, deltaX, deltaY, deltaZ, true, true); }

    /**
     * Evaluate disturbing potential in geocentric coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[in] Z geocentric coordinate of point (meters).
     * @return \e T = \e W - \e U the disturbing potential (also called the
     *   anomalous potential) (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real T(real X, real Y, real Z) const {
      real dummy;
      return InternalT(X, Y, Z, dummy, dummy, dummy, false, true);
    }

    /**
     * Evaluate the components of the acceleration due to normal gravity and
     * the centrifugal acceleration in geocentric coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[in] Z geocentric coordinate of point (meters).
     * @param[out] gammaX the \e X component of the normal acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gammaY the \e Y component of the normal acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gammaZ the \e Z component of the normal acceleration
     *   (m s<sup>&minus;2</sup>).
     * @return \e U = <i>V</i><sub>0</sub> + &Phi; the sum of the
     *   normal gravitational and centrifugal potentials
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     *
     * This calls NormalGravity::U for ReferenceEllipsoid().
     **********************************************************************/
    Math::real U(real X, real Y, real Z,
                 real& gammaX, real& gammaY, real& gammaZ) const
    { return _earth.U(X, Y, Z, gammaX, gammaY, gammaZ); }

    /**
     * Evaluate the centrifugal acceleration in geocentric coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[out] fX the \e X component of the centrifugal acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] fY the \e Y component of the centrifugal acceleration
     *   (m s<sup>&minus;2</sup>).
     * @return &Phi; the centrifugal potential (m<sup>2</sup>
     * s<sup>&minus;2</sup>).
     *
     * This calls NormalGravity::Phi for ReferenceEllipsoid().
     **********************************************************************/
    Math::real Phi(real X, real Y, real& fX, real& fY) const
    { return _earth.Phi(X, Y, fX, fY); }
    ///@}

    /** \name Compute gravity on a circle of constant latitude
     **********************************************************************/
    ///@{
    /**
     * Create a GravityCircle object to allow the gravity field at many points
     * with constant \e lat and \e h and varying \e lon to be computed
     * efficiently.
     *
     * @param[in] lat latitude of the point (degrees).
     * @param[in] h the height of the point above the ellipsoid (meters).
     * @param[in] caps bitor'ed combination of GravityModel::mask values
     *   specifying the capabilities of the resulting GravityCircle object.
     * @exception std::bad_alloc if the memory necessary for creating a
     *   GravityCircle can't be allocated.
     * @return a GravityCircle object whose member functions computes the
     *   gravitational field at a particular values of \e lon.
     *
     * The GravityModel::mask values are
     * - \e caps |= GravityModel::GRAVITY
     * - \e caps |= GravityModel::DISTURBANCE
     * - \e caps |= GravityModel::DISTURBING_POTENTIAL
     * - \e caps |= GravityModel::SPHERICAL_ANOMALY
     * - \e caps |= GravityModel::GEOID_HEIGHT
     * .
     * The default value of \e caps is GravityModel::ALL which turns on all the
     * capabilities.  If an unsupported function is invoked, it will return
     * NaNs.  Note that GravityModel::GEOID_HEIGHT will only be honored if \e h
     * = 0.
     *
     * If the field at several points on a circle of latitude need to be
     * calculated then creating a GravityCircle object and using its member
     * functions will be substantially faster, especially for high-degree
     * models.  See \ref gravityparallel for an example of using GravityCircle
     * (together with OpenMP) to speed up the computation of geoid heights.
     **********************************************************************/
    GravityCircle Circle(real lat, real h, unsigned caps = ALL) const;
    ///@}

    /** \name Inspector functions
     **********************************************************************/
    ///@{

    /**
     * @return the NormalGravity object for the reference ellipsoid.
     **********************************************************************/
    const NormalGravity& ReferenceEllipsoid() const { return _earth; }

    /**
     * @return the description of the gravity model, if available, in the data
     *   file; if absent, return "NONE".
     **********************************************************************/
    const std::string& Description() const { return _description; }

    /**
     * @return date of the model; if absent, return "UNKNOWN".
     **********************************************************************/
    const std::string& DateTime() const { return _date; }

    /**
     * @return full file name used to load the gravity model.
     **********************************************************************/
    const std::string& GravityFile() const { return _filename; }

    /**
     * @return "name" used to load the gravity model (from the first argument
     *   of the constructor, but this may be overridden by the model file).
     **********************************************************************/
    const std::string& GravityModelName() const { return _name; }

    /**
     * @return directory used to load the gravity model.
     **********************************************************************/
    const std::string& GravityModelDirectory() const { return _dir; }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).
     **********************************************************************/
    Math::real MajorRadius() const { return _earth.MajorRadius(); }

    /**
     * @return \e GM the mass constant of the model (m<sup>3</sup>
     *   s<sup>&minus;2</sup>); this is the product of \e G the gravitational
     *   constant and \e M the mass of the earth (usually including the mass of
     *   the earth's atmosphere).
     **********************************************************************/
    Math::real MassConstant() const { return _GMmodel; }

    /**
     * @return \e GM the mass constant of the ReferenceEllipsoid()
     *   (m<sup>3</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real ReferenceMassConstant() const
    { return _earth.MassConstant(); }

    /**
     * @return &omega; the angular velocity of the model and the
     *   ReferenceEllipsoid() (rad s<sup>&minus;1</sup>).
     **********************************************************************/
    Math::real AngularVelocity() const
    { return _earth.AngularVelocity(); }

    /**
     * @return \e f the flattening of the ellipsoid.
     **********************************************************************/
    Math::real Flattening() const { return _earth.Flattening(); }
    ///@}

    /**
     * @return the default path for gravity model data files.
     *
     * This is the value of the environment variable
     * GEOGRAPHICLIB_GRAVITY_PATH, if set; otherwise, it is
     * $GEOGRAPHICLIB_DATA/gravity if the environment variable
     * GEOGRAPHICLIB_DATA is set; otherwise, it is a compile-time default
     * (/usr/local/share/GeographicLib/gravity on non-Windows systems and
     * C:/ProgramData/GeographicLib/gravity on Windows systems).
     **********************************************************************/
    static std::string DefaultGravityPath();

    /**
     * @return the default name for the gravity model.
     *
     * This is the value of the environment variable
     * GEOGRAPHICLIB_GRAVITY_NAME, if set; otherwise, it is "egm96".  The
     * GravityModel class does not use this function; it is just provided as a
     * convenience for a calling program when constructing a GravityModel
     * object.
     **********************************************************************/
    static std::string DefaultGravityName();
  };

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_GRAVITYMODEL_HPP
