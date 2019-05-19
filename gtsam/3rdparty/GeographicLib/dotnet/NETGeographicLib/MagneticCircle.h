#pragma once
/**
 * \file NETGeographicLib/MagneticCircle.h
 * \brief Header for NETGeographicLib::MagneticCircle class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::MagneticCircle.
   *
   * This class allows .NET applications to access GeographicLib::MagneticCircle.
   *
   * Evaluate the earth's magnetic field on a circle of constant height and
   * latitude.  This uses a CircularEngine to pre-evaluate the inner sum of the
   * spherical harmonic sum, allowing the values of the field at several
   * different longitudes to be evaluated rapidly.
   *
   * Use MagneticModel::Circle to create a MagneticCircle object.  (The
   * constructor for this class is for internal use only.)
   *
   * C# Example:
   * \include example-MagneticCircle.cs
   * Managed C++ Example:
   * \include example-MagneticCircle.cpp
   * Visual Basic Example:
   * \include example-MagneticCircle.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The () operator has been replaced with Field.
   *
   * The following functions are implemented as properties:
   * Init, MajorRadius, Flattening, Latitude, Height, and Time.
   **********************************************************************/
    public ref class MagneticCircle
    {
        private:
        // pointer to the unmanaged GeographicLib::MagneticCircle.
        const GeographicLib::MagneticCircle* m_pMagneticCircle;

        // the finalizer frees the unmanaged memory when the object is destroyed.
        !MagneticCircle(void);
    public:

        /**
         * %brief A constructor that is initialized from an unmanaged
         * GeographicLib::MagneticCircle. This is for internal use only.
         *
         * Developers should use MagneticModel::Circle to create a
         * MagneticCircle.
         **********************************************************************/
        MagneticCircle( const GeographicLib::MagneticCircle& c );

        /**
         * %brief The destructor calls the finalizer.
         **********************************************************************/
        ~MagneticCircle()
        { this->!MagneticCircle(); }

        /** \name Compute the magnetic field
         **********************************************************************/
        ///@{
        /**
         * %brief Evaluate the components of the geomagnetic field at a
         * particular longitude.
         *
         * @param[in] lon longitude of the point (degrees).
         * @param[out] Bx the easterly component of the magnetic field (nanotesla).
         * @param[out] By the northerly component of the magnetic field (nanotesla).
         * @param[out] Bz the vertical (up) component of the magnetic field
         *   (nanotesla).
         **********************************************************************/
        void Field(double lon,
            [System::Runtime::InteropServices::Out] double% Bx,
            [System::Runtime::InteropServices::Out] double% By,
            [System::Runtime::InteropServices::Out] double% Bz);

        /**
         * Evaluate the components of the geomagnetic field and their time
         * derivatives at a particular longitude.
         *
         * @param[in] lon longitude of the point (degrees).
         * @param[out] Bx the easterly component of the magnetic field (nanotesla).
         * @param[out] By the northerly component of the magnetic field (nanotesla).
         * @param[out] Bz the vertical (up) component of the magnetic field
         *   (nanotesla).
         * @param[out] Bxt the rate of change of \e Bx (nT/yr).
         * @param[out] Byt the rate of change of \e By (nT/yr).
         * @param[out] Bzt the rate of change of \e Bz (nT/yr).
         **********************************************************************/
        void Field(double lon,
            [System::Runtime::InteropServices::Out] double% Bx,
            [System::Runtime::InteropServices::Out] double% By,
            [System::Runtime::InteropServices::Out] double% Bz,
            [System::Runtime::InteropServices::Out] double% Bxt,
            [System::Runtime::InteropServices::Out] double% Byt,
            [System::Runtime::InteropServices::Out] double% Bzt);
        ///@}

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return true if the object has been initialized.
         **********************************************************************/
        property bool Init { bool get(); }
        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value inherited from the MagneticModel object used in the
         *   constructor.  This property throws a GeographicErr exception if
         *   the object is not initialized.
         **********************************************************************/
        property double MajorRadius { double get(); }
        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the MagneticModel object used in the constructor.
         *   This property throws a GeographicErr exception if the object is
         *   not initialized.
         **********************************************************************/
        property double Flattening { double get(); }
        /**
         * @return the latitude of the circle (degrees).
         *   This property throws a GeographicErr exception if the object is
         *   not initialized.
         **********************************************************************/
        property double Latitude { double get(); }
        /**
         * @return the height of the circle (meters).
         *   This property throws a GeographicErr exception if the object is
         *   not initialized.
         **********************************************************************/
        property double Height { double get(); }
        /**
         * @return the time (fractional years).
         *   This property throws a GeographicErr exception if the object is
         *   not initialized.
         **********************************************************************/
        property double Time { double get(); }
        ///@}
    };
} //namespace NETGeographicLib
