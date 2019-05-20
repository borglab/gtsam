#pragma once
/**
 * \file NETGeographicLib/GravityCircle.h
 * \brief Header for NETGeographicLib::GravityCircle class
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
   * \brief .NET wrapper for GeographicLib::GravityCircle.
   *
   * This class allows .NET applications to access GeographicLib::GravityCircle.
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
   * C# Example:
   * \include example-GravityCircle.cs
   * Managed C++ Example:
   * \include example-GravityCircle.cpp
   * Visual Basic Example:
   * \include example-GravityCircle.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The following functions are implemented as properties:
   * Init, MajorRadius, Flattening, Latitude, and Height.
   *
   * The Capabilities functions accept and return the "capabilities mask"
   * as a NETGeographicLib::GravityModel::Mask rather than an unsigned.
   **********************************************************************/
    public ref class GravityCircle
    {
        private:
        // the pointer to the unmanaged GeographicLib::GravityCircle.
        const GeographicLib::GravityCircle* m_pGravityCircle;

        // the finalizer frees the unmanaged memory when the object is destroyed.
        !GravityCircle(void);
    public:
        /**
         * A constructor that is initialized from an unmanaged
         * GeographicLib::GravityCircle.  For internal use only.  Developers
         * should use GravityModel::Circle to create a GavityCircle object.
         **********************************************************************/
        GravityCircle( const GeographicLib::GravityCircle& gc );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~GravityCircle()
        { this->!GravityCircle(); }

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
         * @return \e W the sum of the gravitational and centrifugal potentials.
         *
         * The function includes the effects of the earth's rotation.
         **********************************************************************/
        double Gravity(double lon,
            [System::Runtime::InteropServices::Out] double% gx,
            [System::Runtime::InteropServices::Out] double% gy,
            [System::Runtime::InteropServices::Out] double% gz);

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
         * @return \e T the corresponding disturbing potential.
         **********************************************************************/
        double Disturbance(double lon,
            [System::Runtime::InteropServices::Out] double% deltax,
            [System::Runtime::InteropServices::Out] double% deltay,
            [System::Runtime::InteropServices::Out] double% deltaz);

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
        double GeoidHeight(double lon);

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
        void SphericalAnomaly(double lon,
            [System::Runtime::InteropServices::Out] double% Dg01,
            [System::Runtime::InteropServices::Out] double% xi,
            [System::Runtime::InteropServices::Out] double% eta);

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
        double W(double lon,
            [System::Runtime::InteropServices::Out] double% gX,
            [System::Runtime::InteropServices::Out] double% gY,
            [System::Runtime::InteropServices::Out] double% gZ);

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
        double V(double lon,
            [System::Runtime::InteropServices::Out] double% GX,
            [System::Runtime::InteropServices::Out] double% GY,
            [System::Runtime::InteropServices::Out] double% GZ);

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
        double T(double lon,
            [System::Runtime::InteropServices::Out] double% deltaX,
            [System::Runtime::InteropServices::Out] double% deltaY,
            [System::Runtime::InteropServices::Out] double% deltaZ);

        /**
         * Evaluate disturbing potential in geocentric coordinates.
         *
         * @param[in] lon the geographic longitude (degrees).
         * @return \e T = \e W - \e U the disturbing potential (also called the
         *   anomalous potential) (m<sup>2</sup> s<sup>&minus;2</sup>).
         **********************************************************************/
        double T(double lon);

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
         *   the value inherited from the GravityModel object used in the
         *   constructor.
         *   This property throws an exception if the GravityCircles has not
         *   been initialized.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the GravityModel object used in the constructor.
         *   This property throws an exception if the GravityCircles has not
         *   been initialized.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * @return the latitude of the circle (degrees).
         *   This property throws an exception if the GravityCircles has not
         *   been initialized.
         **********************************************************************/
        property double Latitude { double get(); }

        /**
         * @return the height of the circle (meters).
         *   This property throws an exception if the GravityCircles has not
         *   been initialized.
         **********************************************************************/
        property double Height { double get(); }

        /**
         * @return \e caps the computational capabilities that this object was
         *   constructed with.
         **********************************************************************/
        GravityModel::Mask Capabilities();

        /**
         * @param[in] testcaps a set of bitor'ed GeodesicLine::mask values.
         * @return true if the GeodesicLine object has all these capabilities.
         **********************************************************************/
        bool Capabilities(GravityModel::Mask testcaps);
        ///@}
    };
} // namespace NETGeographicLib
