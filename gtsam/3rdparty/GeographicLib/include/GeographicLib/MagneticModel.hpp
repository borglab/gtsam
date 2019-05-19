/**
 * \file MagneticModel.hpp
 * \brief Header for GeographicLib::MagneticModel class
 *
 * Copyright (c) Charles Karney (2011-2015) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_MAGNETICMODEL_HPP)
#define GEOGRAPHICLIB_MAGNETICMODEL_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/SphericalHarmonic.hpp>

#if defined(_MSC_VER)
// Squelch warnings about dll vs vector
#  pragma warning (push)
#  pragma warning (disable: 4251)
#endif

namespace GeographicLib {

  class MagneticCircle;

  /**
   * \brief Model of the earth's magnetic field
   *
   * Evaluate the earth's magnetic field according to a model.  At present only
   * internal magnetic fields are handled.  These are due to the earth's code
   * and crust; these vary slowly (over many years).  Excluded are the effects
   * of currents in the ionosphere and magnetosphere which have daily and
   * annual variations.
   *
   * See \ref magnetic for details of how to install the magnetic models and
   * the data format.
   *
   * See
   * - General information:
   *   - http://geomag.org/models/index.html
   * - WMM2010:
   *   - https://ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
   *   - https://ngdc.noaa.gov/geomag/WMM/data/WMM2010/WMM2010COF.zip
   * - WMM2015:
   *   - https://ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
   *   - https://ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015COF.zip
   * - IGRF11:
   *   - https://ngdc.noaa.gov/IAGA/vmod/igrf.html
   *   - https://ngdc.noaa.gov/IAGA/vmod/igrf11coeffs.txt
   *   - https://ngdc.noaa.gov/IAGA/vmod/geomag70_linux.tar.gz
   * - EMM2010:
   *   - https://ngdc.noaa.gov/geomag/EMM/index.html
   *   - https://ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2010_Sph_Windows_Linux.zip
   * - EMM2015:
   *   - https://ngdc.noaa.gov/geomag/EMM/index.html
   *   - https://www.ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2015_Sph_Linux.zip
   * - EMM2017:
   *   - https://ngdc.noaa.gov/geomag/EMM/index.html
   *   - https://www.ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2017_Sph_Linux.zip
   *
   * Example of use:
   * \include example-MagneticModel.cpp
   *
   * <a href="MagneticField.1.html">MagneticField</a> is a command-line utility
   * providing access to the functionality of MagneticModel and MagneticCircle.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT MagneticModel {
  private:
    typedef Math::real real;
    static const int idlength_ = 8;
    std::string _name, _dir, _description, _date, _filename, _id;
    real _t0, _dt0, _tmin, _tmax, _a, _hmin, _hmax;
    int _Nmodels, _Nconstants;
    SphericalHarmonic::normalization _norm;
    Geocentric _earth;
    std::vector< std::vector<real> > _G;
    std::vector< std::vector<real> > _H;
    std::vector<SphericalHarmonic> _harm;
    void Field(real t, real lat, real lon, real h, bool diffp,
               real& Bx, real& By, real& Bz,
               real& Bxt, real& Byt, real& Bzt) const;
    void ReadMetadata(const std::string& name);
    MagneticModel(const MagneticModel&); // copy constructor not allowed
    MagneticModel& operator=(const MagneticModel&); // nor copy assignment
  public:

    /** \name Setting up the magnetic model
     **********************************************************************/
    ///@{
    /**
     * Construct a magnetic model.
     *
     * @param[in] name the name of the model.
     * @param[in] path (optional) directory for data file.
     * @param[in] earth (optional) Geocentric object for converting
     *   coordinates; default Geocentric::WGS84().
     * @exception GeographicErr if the data file cannot be found, is
     *   unreadable, or is corrupt.
     * @exception std::bad_alloc if the memory necessary for storing the model
     *   can't be allocated.
     *
     * A filename is formed by appending ".wmm" (World Magnetic Model) to the
     * name.  If \e path is specified (and is non-empty), then the file is
     * loaded from directory, \e path.  Otherwise the path is given by the
     * DefaultMagneticPath().
     *
     * This file contains the metadata which specifies the properties of the
     * model.  The coefficients for the spherical harmonic sums are obtained
     * from a file obtained by appending ".cof" to metadata file (so the
     * filename ends in ".wwm.cof").
     *
     * The model is not tied to a particular ellipsoidal model of the earth.
     * The final earth argument to the constructor specifies an ellipsoid to
     * allow geodetic coordinates to the transformed into the spherical
     * coordinates used in the spherical harmonic sum.
     **********************************************************************/
    explicit MagneticModel(const std::string& name,
                           const std::string& path = "",
                           const Geocentric& earth = Geocentric::WGS84());
    ///@}

    /** \name Compute the magnetic field
     **********************************************************************/
    ///@{
    /**
     * Evaluate the components of the geomagnetic field.
     *
     * @param[in] t the time (years).
     * @param[in] lat latitude of the point (degrees).
     * @param[in] lon longitude of the point (degrees).
     * @param[in] h the height of the point above the ellipsoid (meters).
     * @param[out] Bx the easterly component of the magnetic field (nanotesla).
     * @param[out] By the northerly component of the magnetic field
     *   (nanotesla).
     * @param[out] Bz the vertical (up) component of the magnetic field
     *   (nanotesla).
     **********************************************************************/
    void operator()(real t, real lat, real lon, real h,
                    real& Bx, real& By, real& Bz) const {
      real dummy;
      Field(t, lat, lon, h, false, Bx, By, Bz, dummy, dummy, dummy);
    }

    /**
     * Evaluate the components of the geomagnetic field and their time
     * derivatives
     *
     * @param[in] t the time (years).
     * @param[in] lat latitude of the point (degrees).
     * @param[in] lon longitude of the point (degrees).
     * @param[in] h the height of the point above the ellipsoid (meters).
     * @param[out] Bx the easterly component of the magnetic field (nanotesla).
     * @param[out] By the northerly component of the magnetic field
     *   (nanotesla).
     * @param[out] Bz the vertical (up) component of the magnetic field
     *   (nanotesla).
     * @param[out] Bxt the rate of change of \e Bx (nT/yr).
     * @param[out] Byt the rate of change of \e By (nT/yr).
     * @param[out] Bzt the rate of change of \e Bz (nT/yr).
     **********************************************************************/
    void operator()(real t, real lat, real lon, real h,
                    real& Bx, real& By, real& Bz,
                    real& Bxt, real& Byt, real& Bzt) const {
      Field(t, lat, lon, h, true, Bx, By, Bz, Bxt, Byt, Bzt);
    }

    /**
     * Create a MagneticCircle object to allow the geomagnetic field at many
     * points with constant \e lat, \e h, and \e t and varying \e lon to be
     * computed efficiently.
     *
     * @param[in] t the time (years).
     * @param[in] lat latitude of the point (degrees).
     * @param[in] h the height of the point above the ellipsoid (meters).
     * @exception std::bad_alloc if the memory necessary for creating a
     *   MagneticCircle can't be allocated.
     * @return a MagneticCircle object whose MagneticCircle::operator()(real
     *   lon) member function computes the field at particular values of \e
     *   lon.
     *
     * If the field at several points on a circle of latitude need to be
     * calculated then creating a MagneticCircle and using its member functions
     * will be substantially faster, especially for high-degree models.
     **********************************************************************/
    MagneticCircle Circle(real t, real lat, real h) const;

    /**
     * Compute various quantities dependent on the magnetic field.
     *
     * @param[in] Bx the \e x (easterly) component of the magnetic field (nT).
     * @param[in] By the \e y (northerly) component of the magnetic field (nT).
     * @param[in] Bz the \e z (vertical, up positive) component of the magnetic
     *   field (nT).
     * @param[out] H the horizontal magnetic field (nT).
     * @param[out] F the total magnetic field (nT).
     * @param[out] D the declination of the field (degrees east of north).
     * @param[out] I the inclination of the field (degrees down from
     *   horizontal).
     **********************************************************************/
    static void FieldComponents(real Bx, real By, real Bz,
                                real& H, real& F, real& D, real& I) {
      real Ht, Ft, Dt, It;
      FieldComponents(Bx, By, Bz, real(0), real(1), real(0),
                      H, F, D, I, Ht, Ft, Dt, It);
    }

    /**
     * Compute various quantities dependent on the magnetic field and its rate
     * of change.
     *
     * @param[in] Bx the \e x (easterly) component of the magnetic field (nT).
     * @param[in] By the \e y (northerly) component of the magnetic field (nT).
     * @param[in] Bz the \e z (vertical, up positive) component of the magnetic
     *   field (nT).
     * @param[in] Bxt the rate of change of \e Bx (nT/yr).
     * @param[in] Byt the rate of change of \e By (nT/yr).
     * @param[in] Bzt the rate of change of \e Bz (nT/yr).
     * @param[out] H the horizontal magnetic field (nT).
     * @param[out] F the total magnetic field (nT).
     * @param[out] D the declination of the field (degrees east of north).
     * @param[out] I the inclination of the field (degrees down from
     *   horizontal).
     * @param[out] Ht the rate of change of \e H (nT/yr).
     * @param[out] Ft the rate of change of \e F (nT/yr).
     * @param[out] Dt the rate of change of \e D (degrees/yr).
     * @param[out] It the rate of change of \e I (degrees/yr).
     **********************************************************************/
    static void FieldComponents(real Bx, real By, real Bz,
                                real Bxt, real Byt, real Bzt,
                                real& H, real& F, real& D, real& I,
                                real& Ht, real& Ft, real& Dt, real& It);
    ///@}

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return the description of the magnetic model, if available, from the
     *   Description file in the data file; if absent, return "NONE".
     **********************************************************************/
    const std::string& Description() const { return _description; }

    /**
     * @return date of the model, if available, from the ReleaseDate field in
     *   the data file; if absent, return "UNKNOWN".
     **********************************************************************/
    const std::string& DateTime() const { return _date; }

    /**
     * @return full file name used to load the magnetic model.
     **********************************************************************/
    const std::string& MagneticFile() const { return _filename; }

    /**
     * @return "name" used to load the magnetic model (from the first argument
     *   of the constructor, but this may be overridden by the model file).
     **********************************************************************/
    const std::string& MagneticModelName() const { return _name; }

    /**
     * @return directory used to load the magnetic model.
     **********************************************************************/
    const std::string& MagneticModelDirectory() const { return _dir; }

    /**
     * @return the minimum height above the ellipsoid (in meters) for which
     *   this MagneticModel should be used.
     *
     * Because the model will typically provide useful results
     * slightly outside the range of allowed heights, no check of \e t
     * argument is made by MagneticModel::operator()() or
     * MagneticModel::Circle.
     **********************************************************************/
    Math::real MinHeight() const { return _hmin; }

    /**
     * @return the maximum height above the ellipsoid (in meters) for which
     *   this MagneticModel should be used.
     *
     * Because the model will typically provide useful results
     * slightly outside the range of allowed heights, no check of \e t
     * argument is made by MagneticModel::operator()() or
     * MagneticModel::Circle.
     **********************************************************************/
    Math::real MaxHeight() const { return _hmax; }

    /**
     * @return the minimum time (in years) for which this MagneticModel should
     *   be used.
     *
     * Because the model will typically provide useful results
     * slightly outside the range of allowed times, no check of \e t
     * argument is made by MagneticModel::operator()() or
     * MagneticModel::Circle.
     **********************************************************************/
    Math::real MinTime() const { return _tmin; }

    /**
     * @return the maximum time (in years) for which this MagneticModel should
     *   be used.
     *
     * Because the model will typically provide useful results
     * slightly outside the range of allowed times, no check of \e t
     * argument is made by MagneticModel::operator()() or
     * MagneticModel::Circle.
     **********************************************************************/
    Math::real MaxTime() const { return _tmax; }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value of \e a inherited from the Geocentric object used in the
     *   constructor.
     **********************************************************************/
    Math::real MajorRadius() const { return _earth.MajorRadius(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the Geocentric object used in the constructor.
     **********************************************************************/
    Math::real Flattening() const { return _earth.Flattening(); }
    ///@}

    /**
     * @return the default path for magnetic model data files.
     *
     * This is the value of the environment variable
     * GEOGRAPHICLIB_MAGNETIC_PATH, if set; otherwise, it is
     * $GEOGRAPHICLIB_DATA/magnetic if the environment variable
     * GEOGRAPHICLIB_DATA is set; otherwise, it is a compile-time default
     * (/usr/local/share/GeographicLib/magnetic on non-Windows systems and
     * C:/ProgramData/GeographicLib/magnetic on Windows systems).
     **********************************************************************/
    static std::string DefaultMagneticPath();

    /**
     * @return the default name for the magnetic model.
     *
     * This is the value of the environment variable
     * GEOGRAPHICLIB_MAGNETIC_NAME, if set; otherwise, it is "wmm2015".  The
     * MagneticModel class does not use this function; it is just provided as a
     * convenience for a calling program when constructing a MagneticModel
     * object.
     **********************************************************************/
    static std::string DefaultMagneticName();
  };

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_MAGNETICMODEL_HPP
