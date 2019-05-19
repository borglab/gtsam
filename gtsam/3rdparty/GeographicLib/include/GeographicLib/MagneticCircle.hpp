/**
 * \file MagneticCircle.hpp
 * \brief Header for GeographicLib::MagneticCircle class
 *
 * Copyright (c) Charles Karney (2011-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_MAGNETICCIRCLE_HPP)
#define GEOGRAPHICLIB_MAGNETICCIRCLE_HPP 1

#include <vector>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/CircularEngine.hpp>

namespace GeographicLib {

  /**
   * \brief Geomagnetic field on a circle of latitude
   *
   * Evaluate the earth's magnetic field on a circle of constant height and
   * latitude.  This uses a CircularEngine to pre-evaluate the inner sum of the
   * spherical harmonic sum, allowing the values of the field at several
   * different longitudes to be evaluated rapidly.
   *
   * Use MagneticModel::Circle to create a MagneticCircle object.  (The
   * constructor for this class is private.)
   *
   * Example of use:
   * \include example-MagneticCircle.cpp
   *
   * <a href="MagneticField.1.html">MagneticField</a> is a command-line utility
   * providing access to the functionality of MagneticModel and MagneticCircle.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT MagneticCircle {
  private:
    typedef Math::real real;

    real _a, _f, _lat, _h, _t, _cphi, _sphi, _t1, _dt0;
    bool _interpolate, _constterm;
    CircularEngine _circ0, _circ1, _circ2;

    MagneticCircle(real a, real f, real lat, real h, real t,
                   real cphi, real sphi, real t1, real dt0,
                   bool interpolate,
                   const CircularEngine& circ0, const CircularEngine& circ1)
      : _a(a)
      , _f(f)
      , _lat(Math::LatFix(lat))
      , _h(h)
      , _t(t)
      , _cphi(cphi)
      , _sphi(sphi)
      , _t1(t1)
      , _dt0(dt0)
      , _interpolate(interpolate)
      , _constterm(false)
      , _circ0(circ0)
      , _circ1(circ1)
    {}

    MagneticCircle(real a, real f, real lat, real h, real t,
                   real cphi, real sphi, real t1, real dt0,
                   bool interpolate,
                   const CircularEngine& circ0, const CircularEngine& circ1,
                   const CircularEngine& circ2)
      : _a(a)
      , _f(f)
      , _lat(lat)
      , _h(h)
      , _t(t)
      , _cphi(cphi)
      , _sphi(sphi)
      , _t1(t1)
      , _dt0(dt0)
      , _interpolate(interpolate)
      , _constterm(true)
      , _circ0(circ0)
      , _circ1(circ1)
      , _circ2(circ2)
    {}

    void Field(real lon, bool diffp,
               real& Bx, real& By, real& Bz,
               real& Bxt, real& Byt, real& Bzt) const;

    friend class MagneticModel; // MagneticModel calls the private constructor

  public:

    /**
     * A default constructor for the normal gravity.  This sets up an
     * uninitialized object which can be later replaced by the
     * MagneticModel::Circle.
     **********************************************************************/
    MagneticCircle() : _a(-1) {}

    /** \name Compute the magnetic field
     **********************************************************************/
    ///@{
    /**
     * Evaluate the components of the geomagnetic field at a particular
     * longitude.
     *
     * @param[in] lon longitude of the point (degrees).
     * @param[out] Bx the easterly component of the magnetic field (nanotesla).
     * @param[out] By the northerly component of the magnetic field
     *   (nanotesla).
     * @param[out] Bz the vertical (up) component of the magnetic field
     *   (nanotesla).
     **********************************************************************/
    void operator()(real lon, real& Bx, real& By, real& Bz) const {
      real dummy;
      Field(lon, false, Bx, By, Bz, dummy, dummy, dummy);
    }

    /**
     * Evaluate the components of the geomagnetic field and their time
     * derivatives at a particular longitude.
     *
     * @param[in] lon longitude of the point (degrees).
     * @param[out] Bx the easterly component of the magnetic field (nanotesla).
     * @param[out] By the northerly component of the magnetic field
     *   (nanotesla).
     * @param[out] Bz the vertical (up) component of the magnetic field
     *   (nanotesla).
     * @param[out] Bxt the rate of change of \e Bx (nT/yr).
     * @param[out] Byt the rate of change of \e By (nT/yr).
     * @param[out] Bzt the rate of change of \e Bz (nT/yr).
     **********************************************************************/
    void operator()(real lon, real& Bx, real& By, real& Bz,
                    real& Bxt, real& Byt, real& Bzt) const {
      Field(lon, true, Bx, By, Bz, Bxt, Byt, Bzt);
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
     *   the value inherited from the MagneticModel object used in the
     *   constructor.
     **********************************************************************/
    Math::real MajorRadius() const
    { return Init() ? _a : Math::NaN(); }
    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the MagneticModel object used in the constructor.
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
     * @return the time (fractional years).
     **********************************************************************/
    Math::real Time() const
    { return Init() ? _t : Math::NaN(); }

    ///@}
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_MAGNETICCIRCLE_HPP
