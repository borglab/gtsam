#pragma once
/**
 * \file NETGeographicLib/MagneticModel.h
 * \brief Header for NETGeographicLib::MagneticModel class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    ref class MagneticCircle;
    ref class Geocentric;
  /**
   * \brief .NET wrapper for GeographicLib::MagneticModel.
   *
   * This class allows .NET applications to access GeographicLib::MagneticModel.
   *
   * Evaluate the earth's magnetic field according to a model.  At present only
   * internal magnetic fields are handled.  These are due to the earth's code
   * and crust; these vary slowly (over many years).  Excluded are the effects
   * of currents in the ionosphere and magnetosphere which have daily and
   * annual variations.
   *
   * See \ref magnetic for details of how to install the magnetic model and the
   * data format.
   *
   * See
   * - General information:
   *   - http://geomag.org/models/index.html
   * - WMM2010:
   *   - http://ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
   *   - http://ngdc.noaa.gov/geomag/WMM/data/WMM2010/WMM2010COF.zip
   * - WMM2015:
   *   - http://ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
   *   - http://ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015COF.zip
   * - IGRF11:
   *   - http://ngdc.noaa.gov/IAGA/vmod/igrf.html
   *   - http://ngdc.noaa.gov/IAGA/vmod/igrf11coeffs.txt
   *   - http://ngdc.noaa.gov/IAGA/vmod/geomag70_linux.tar.gz
   * - EMM2010:
   *   - http://ngdc.noaa.gov/geomag/EMM/index.html
   *   - http://ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2010_Sph_Windows_Linux.zip
   *
   * C# Example:
   * \include example-MagneticModel.cs
   * Managed C++ Example:
   * \include example-MagneticModel.cpp
   * Visual Basic Example:
   * \include example-MagneticModel.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The () operator has been replaced with Field.
   *
   * The following functions are implemented as properties:
   * Description, DateTime, MagneticFile, MagneticModelName,
   * MagneticModelDirectory, MinHeight, MaxHeight, MinTime, MaxTime,
   * MajorRadius, and Flattening.
   **********************************************************************/
    public ref class MagneticModel
    {
        private:
        // The pointer to the unmanaged GeographicLib::MagneticModel.
        const GeographicLib::MagneticModel* m_pMagneticModel;

        // The finalizer frees the unmanaged memory when the object is destroyed.
        !MagneticModel(void);
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
         *   coordinates.
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
        MagneticModel(System::String^ name,
                      System::String^ path,
                      Geocentric^ earth);
        /**
         * Construct a magnetic model that assumes the WGS84 ellipsoid.
         *
         * @param[in] name the name of the model.
         * @param[in] path (optional) directory for data file.
         * @exception GeographicErr if the data file cannot be found, is
         *   unreadable, or is corrupt.
         * @exception GeographicErr if the memory necessary for storing the model
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
        MagneticModel(System::String^ name,
                      System::String^ path);

        ///@}
        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~MagneticModel()
        { this->!MagneticModel(); }

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
         * @param[out] By the northerly component of the magnetic field (nanotesla).
         * @param[out] Bz the vertical (up) component of the magnetic field
         *   (nanotesla).
         **********************************************************************/
        void Field(double t, double lat, double lon, double h,
                        [System::Runtime::InteropServices::Out] double% Bx,
                        [System::Runtime::InteropServices::Out] double% By,
                        [System::Runtime::InteropServices::Out] double% Bz);

        /**
         * Evaluate the components of the geomagnetic field and their time
         * derivatives
         *
         * @param[in] t the time (years).
         * @param[in] lat latitude of the point (degrees).
         * @param[in] lon longitude of the point (degrees).
         * @param[in] h the height of the point above the ellipsoid (meters).
         * @param[out] Bx the easterly component of the magnetic field (nanotesla).
         * @param[out] By the northerly component of the magnetic field (nanotesla).
         * @param[out] Bz the vertical (up) component of the magnetic field
         *   (nanotesla).
         * @param[out] Bxt the rate of change of \e Bx (nT/yr).
         * @param[out] Byt the rate of change of \e By (nT/yr).
         * @param[out] Bzt the rate of change of \e Bz (nT/yr).
         **********************************************************************/
        void Field(double t, double lat, double lon, double h,
                        [System::Runtime::InteropServices::Out] double% Bx,
                        [System::Runtime::InteropServices::Out] double% By,
                        [System::Runtime::InteropServices::Out] double% Bz,
                        [System::Runtime::InteropServices::Out] double% Bxt,
                        [System::Runtime::InteropServices::Out] double% Byt,
                        [System::Runtime::InteropServices::Out] double% Bzt);

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
         * @return a MagneticCircle object whose MagneticCircle::Field(double
         *   lon) member function computes the field at particular values of \e
         *   lon.
         *
         * If the field at several points on a circle of latitude need to be
         * calculated then creating a MagneticCircle and using its member functions
         * will be substantially faster, especially for high-degree models.
         **********************************************************************/
        MagneticCircle^ Circle(double t, double lat, double h);

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
        static void FieldComponents(double Bx, double By, double Bz,
                        [System::Runtime::InteropServices::Out] double% H,
                        [System::Runtime::InteropServices::Out] double% F,
                        [System::Runtime::InteropServices::Out] double% D,
                        [System::Runtime::InteropServices::Out] double% I);

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
        static void FieldComponents(double Bx, double By, double Bz,
                                    double Bxt, double Byt, double Bzt,
                        [System::Runtime::InteropServices::Out] double% H,
                        [System::Runtime::InteropServices::Out] double% F,
                        [System::Runtime::InteropServices::Out] double% D,
                        [System::Runtime::InteropServices::Out] double% I,
                        [System::Runtime::InteropServices::Out] double% Ht,
                        [System::Runtime::InteropServices::Out] double% Ft,
                        [System::Runtime::InteropServices::Out] double% Dt,
                        [System::Runtime::InteropServices::Out] double% It);
        ///@}

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return the description of the magnetic model, if available, from the
         *   Description file in the data file; if absent, return "NONE".
         **********************************************************************/
        property System::String^ Description { System::String^ get(); }

        /**
         * @return date of the model, if available, from the ReleaseDate field in
         *   the data file; if absent, return "UNKNOWN".
         **********************************************************************/
        property System::String^ DateTime { System::String^ get(); }

        /**
         * @return full file name used to load the magnetic model.
         **********************************************************************/
        property System::String^ MagneticFile { System::String^ get(); }

        /**
         * @return "name" used to load the magnetic model (from the first argument
         *   of the constructor, but this may be overridden by the model file).
         **********************************************************************/
        property System::String^ MagneticModelName { System::String^ get(); }

        /**
         * @return directory used to load the magnetic model.
         **********************************************************************/
        property System::String^ MagneticModelDirectory { System::String^ get(); }

        /**
         * @return the minimum height above the ellipsoid (in meters) for which
         *   this MagneticModel should be used.
         *
         * Because the model will typically provide useful results
         * slightly outside the range of allowed heights, no check of \e t
         * argument is made by MagneticModel::Field() or
         * MagneticModel::Circle.
         **********************************************************************/
        property double MinHeight { double get(); }

        /**
         * @return the maximum height above the ellipsoid (in meters) for which
         *   this MagneticModel should be used.
         *
         * Because the model will typically provide useful results
         * slightly outside the range of allowed heights, no check of \e t
         * argument is made by MagneticModel::Field() or
         * MagneticModel::Circle.
         **********************************************************************/
        property double MaxHeight { double get(); }

        /**
         * @return the minimum time (in years) for which this MagneticModel should
         *   be used.
         *
         * Because the model will typically provide useful results
         * slightly outside the range of allowed times, no check of \e t
         * argument is made by MagneticModel::Field() or
         * MagneticModel::Circle.
         **********************************************************************/
        property double MinTime { double get(); }

        /**
         * @return the maximum time (in years) for which this MagneticModel should
         *   be used.
         *
         * Because the model will typically provide useful results
         * slightly outside the range of allowed times, no check of \e t
         * argument is made by MagneticModel::Field() or
         * MagneticModel::Circle.
         **********************************************************************/
        property double MaxTime { double get(); }

        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value of \e a inherited from the Geocentric object used in the
         *   constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the Geocentric object used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }
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
        static System::String^ DefaultMagneticPath();

        /**
         * @return the default name for the magnetic model.
         *
         * This is the value of the environment variable
         * GEOGRAPHICLIB_MAGNETIC_NAME, if set, otherwise, it is "wmm2015".
         * The MagneticModel class does not use this function; it is just
         * provided as a convenience for a calling program when constructing a
         * MagneticModel object.
         **********************************************************************/
        static System::String^ DefaultMagneticName();
    };
} //namespace NETGeographicLib
