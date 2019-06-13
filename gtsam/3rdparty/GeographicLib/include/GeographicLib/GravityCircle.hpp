/**
 * \file GravityCircle.hpp
 * \brief Header for GeographicLib::GravityCircle class
 *
 * Copyright (c) Charles Karney (2011-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GRAVITYCIRCLE_HPP)
#define GEOGRAPHICLIB_GRAVITYCIRCLE_HPP 1

#include <vector>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/CircularEngine.hpp>
#include <GeographicLib/GravityModel.hpp>

namespace GeographicLib {

  /**
   * \brief Gravity on a circle of latitude
   *
   * Evaluate the earth's gravity field on a circle of constant height and
   * latitude.  This uses a CircularEngine to pre-evaluate the inner sum of the
   * spherical harmonic sum, allowing the values of the field at several
   * different longitudes to be evaluated rapidly.
   *
   * Use GravityModel::Circle to create a GravityCircle object.  (The
   * constructor for this class is private.)
   *
   * See \ref gravityparallel for an example of using GravityCircle (together
   * with OpenMP) to speed up the computation of geoid heights.
   *
   * Example of use:
   * \include example-GravityCircle.cpp
   *
   * <a href="Gravity.1.html">Gravity</a> is a command-line utility providing
   * access to the functionality of GravityModel and GravityCircle.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT GravityCircle {
  private:
    typedef Math::real real;
    enum mask {
      NONE                 = GravityModel::NONE,
      GRAVITY              = GravityModel::GRAVITY,
      DISTURBANCE          = GravityModel::DISTURBANCE,
      DISTURBING_POTENTIAL = GravityModel::DISTURBING_POTENTIAL,
      GEOID_HEIGHT         = GravityModel::GEOID_HEIGHT,
      SPHERICAL_ANOMALY    = GravityModel::SPHERICAL_ANOMALY,
      ALL                  = GravityModel::ALL,
    };

    unsigned _caps;
    real _a, _f, _lat, _h, _Z, _Px, _invR, _cpsi, _spsi,
      _cphi, _sphi, _amodel, _GMmodel, _dzonal0,
      _corrmult, _gamma0, _gamma, _frot;
    CircularEngine _gravitational, _disturbing, _correction;

    GravityCircle(mask caps, real a, real f, real lat, real h,
                  real Z, real P, real cphi, real sphi,
                  real amodel, real GMmodel, real dzonal0, real corrmult,
                  real gamma0, real gamma, real frot,
                  const CircularEngine& gravitational,
                  const CircularEngine& disturbing,
                  const CircularEngine& correction)
      : _caps(caps)
      , _a(a)
      , _f(f)
      , _lat(Math::LatFix(lat))
      , _h(h)
      , _Z(Z)
      , _Px(P)
      , _invR(1 / Math::hypot(_Px, _Z))
      , _cpsi(_Px * _invR)
      , _spsi(_Z * _invR)
      , _cphi(cphi)
      , _sphi(sphi)
      , _amodel(amodel)
      , _GMmodel(GMmodel)
      , _dzonal0(dzonal0)
      , _corrmult(corrmult)
      , _gamma0(gamma0)
      , _gamma(gamma)
      , _frot(frot)
      , _gravitational(gravitational)
      , _disturbing(disturbing)
      , _correction(correction)
    {}

    friend class GravityModel; // GravityModel calls the private constructor
    Math::real W(real slam, real clam,
                 real& gX, real& gY, real& gZ) const;
    Math::real V(real slam, real clam,
                 real& gX, real& gY, real& gZ) const;
    Math::real InternalT(real slam, real clam,
                         real& deltaX, real& deltaY, real& deltaZ,
                         bool gradp, bool correct) const;
  public:
    /**
     * A default constructor for the normal gravity.  This sets up an
     * uninitialized object which can be later replaced by the
     * GravityModel::Circle.
     **********************************************************************/
    GravityCircle() : _a(-1) {}

    /** \name Compute the gravitational field
     **********************************************************************/
    ///@{
    /**
     * Evaluate the gravity.
     *
     * @param[in] lon the geographic longitude (degrees).
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
    Math::real Gravity(real lon, real& gx, real& gy, real& gz) const;

    /**
     * Evaluate the gravity disturbance vector.
     *
     * @param[in] lon the geographic longitude (degrees).
     * @param[out] deltax the easterly component of the disturbance vector
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltay the northerly component of the disturbance vector
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltaz the upward component of the disturbance vector
     *   (m s<sup>&minus;2</sup>).
     * @return \e T the corresponding disturbing potential
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real Disturbance(real lon, real& deltax, real& deltay, real& deltaz)
      const;

    /**
     * Evaluate the geoid height.
     *
     * @param[in] lon the geographic longitude (degrees).
     * @return \e N the height of the geoid above the reference ellipsoid
     *   (meters).
     *
     * Some approximations are made in computing the geoid height so that the
     * results of the NGA codes are reproduced accurately.  Details are given
     * in \ref gravitygeoid.
     **********************************************************************/
    Math::real GeoidHeight(real lon) const;

    /**
     * Evaluate the components of the gravity anomaly vector using the
     * spherical approximation.
     *
     * @param[in] lon the geographic longitude (degrees).
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
    void SphericalAnomaly(real lon, real& Dg01, real& xi, real& eta)
      const;

    /**
     * Evaluate the components of the acceleration due to gravity and the
     * centrifugal acceleration in geocentric coordinates.
     *
     * @param[in] lon the geographic longitude (degrees).
     * @param[out] gX the \e X component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gY the \e Y component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] gZ the \e Z component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @return \e W = \e V + &Phi; the sum of the gravitational and
     *   centrifugal potentials (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real W(real lon, real& gX, real& gY, real& gZ) const {
      real slam, clam;
      Math::sincosd(lon, slam, clam);
      return W(slam, clam, gX, gY, gZ);
    }

    /**
     * Evaluate the components of the acceleration due to gravity in geocentric
     * coordinates.
     *
     * @param[in] lon the geographic longitude (degrees).
     * @param[out] GX the \e X component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] GY the \e Y component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @param[out] GZ the \e Z component of the acceleration
     *   (m s<sup>&minus;2</sup>).
     * @return \e V = \e W - &Phi; the gravitational potential
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real V(real lon, real& GX, real& GY, real& GZ) const {
      real slam, clam;
      Math::sincosd(lon, slam, clam);
      return V(slam, clam, GX, GY, GZ);
    }

    /**
     * Evaluate the components of the gravity disturbance in geocentric
     * coordinates.
     *
     * @param[in] lon the geographic longitude (degrees).
     * @param[out] deltaX the \e X component of the gravity disturbance
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltaY the \e Y component of the gravity disturbance
     *   (m s<sup>&minus;2</sup>).
     * @param[out] deltaZ the \e Z component of the gravity disturbance
     *   (m s<sup>&minus;2</sup>).
     * @return \e T = \e W - \e U the disturbing potential (also called the
     *   anomalous potential) (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real T(real lon, real& deltaX, real& deltaY, real& deltaZ)
      const {
      real slam, clam;
      Math::sincosd(lon, slam, clam);
      return InternalT(slam, clam, deltaX, deltaY, deltaZ, true, true);
    }

    /**
     * Evaluate disturbing potential in geocentric coordinates.
     *
     * @param[in] lon the geographic longitude (degrees).
     * @return \e T = \e W - \e U the disturbing potential (also called the
     *   anomalous potential) (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real T(real lon) const {
      real slam, clam, dummy;
      Math::sincosd(lon, slam, clam);
      return InternalT(slam, clam, dummy, dummy, dummy, false, true);
    }

    ///@}

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return true if the object has been initialized.
     **********************************************************************/
    bool Init() const { return _a > 0; }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value inherited from the GravityModel object used in the
     *   constructor.
     **********************************************************************/
    Math::real MajorRadius() const
    { return Init() ? _a : Math::NaN(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the GravityModel object used in the constructor.
     **********************************************************************/
    Math::real Flattening() const
    { return Init() ? _f : Math::NaN(); }

    /**
     * @return the latitude of the circle (degrees).
     **********************************************************************/
    Math::real Latitude() const
    { return Init() ? _lat : Math::NaN(); }

    /**
     * @return the height of the circle (meters).
     **********************************************************************/
    Math::real Height() const
    { return Init() ? _h : Math::NaN(); }

    /**
     * @return \e caps the computational capabilities that this object was
     *   constructed with.
     **********************************************************************/
    unsigned Capabilities() const { return _caps; }

    /**
     * @param[in] testcaps a set of bitor'ed GravityModel::mask values.
     * @return true if the GravityCircle object has all these capabilities.
     **********************************************************************/
    bool Capabilities(unsigned testcaps) const {
      return (_caps & testcaps) == testcaps;
    }
    ///@}
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GRAVITYCIRCLE_HPP
