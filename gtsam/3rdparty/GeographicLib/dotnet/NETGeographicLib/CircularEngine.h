/**
 * \file NETGeographicLib/CircularEngine.h
 * \brief Header for NETGeographicLib::CircularEngine class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#pragma once

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::CircularEngine.
   *
   * This class allows .NET applications to access GeographicLib::CircularEngine.
   *
   * The class is a companion to SphericalEngine.  If the results of a
   * spherical harmonic sum are needed for several points on a circle of
   * constant latitude \e lat and height \e h, then SphericalEngine::Circle can
   * compute the inner sum, which is independent of longitude \e lon, and
   * produce a CircularEngine object.  CircularEngine::LongitudeSum() can
   * then be used to perform the outer sum for particular values of \e lon.
   * This can lead to substantial improvements in computational speed for high
   * degree sum (approximately by a factor of \e N / 2 where \e N is the
   * maximum degree).
   *
   * CircularEngine is tightly linked to the internals of SphericalEngine.  For
   * that reason, the constructor for this class is for internal use only.  Use
   * SphericalHarmonic::Circle, SphericalHarmonic1::Circle, and
   * SphericalHarmonic2::Circle to create instances of this class.
   *
   * CircularEngine stores the coefficients needed to allow the summation over
   * order to be performed in 2 or 6 vectors of length \e M + 1 (depending on
   * whether gradients are to be calculated).  For this reason the constructor
   * may throw a GeographicErr exception.
   *
   * C# Example:
   * \include example-CircularEngine.cs
   * Managed C++ Example:
   * \include example-CircularEngine.cpp
   * Visual Basic Example:
   * \include example-CircularEngine.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * The () operator has been replaced with with LongitudeSum.
   *
   * This class does not have a constructor that can be used in a .NET
   * application.  Use SphericalHarmonic::Circle, SphericalHarmonic1::Circle or
   * SphericalHarmonic2::Circle to create instances of this class.
   **********************************************************************/
    public ref class CircularEngine
    {
        private:
        // pointer to the unmanaged GeographicLib::CircularEngine
        const GeographicLib::CircularEngine* m_pCircularEngine;

        // The finalizer frees the unmanaged memory when the object is destroyed.
        !CircularEngine();
    public:
        /**
         * The constructor.
         *
         * This constructor should not be used in .NET applications.
         * Use SphericalHarmonic::Circle, SphericalHarmonic1::Circle or
         * SphericalHarmonic2::Circle to create instances of this class.
         *
         * @param[in] c The unmanaged CircularEngine to be copied.
         **********************************************************************/
        CircularEngine( const GeographicLib::CircularEngine& c );

        /**
         * The destructor calls the finalizer
         **********************************************************************/
        ~CircularEngine()
        { this->!CircularEngine(); }

        /**
         * Evaluate the sum for a particular longitude given in terms of its
         * cosine and sine.
         *
         * @param[in] coslon the cosine of the longitude.
         * @param[in] sinlon the sine of the longitude.
         * @return \e V the value of the sum.
         *
         * The arguments must satisfy <i>coslon</i><sup>2</sup> +
         * <i>sinlon</i><sup>2</sup> = 1.
         **********************************************************************/
        double LongitudeSum(double coslon, double sinlon);

        /**
         * Evaluate the sum for a particular longitude.
         *
         * @param[in] lon the longitude (degrees).
         * @return \e V the value of the sum.
         **********************************************************************/
        double LongitudeSum(double lon);

        /**
         * Evaluate the sum and its gradient for a particular longitude given in
         * terms of its cosine and sine.
         *
         * @param[in] coslon the cosine of the longitude.
         * @param[in] sinlon the sine of the longitude.
         * @param[out] gradx \e x component of the gradient.
         * @param[out] grady \e y component of the gradient.
         * @param[out] gradz \e z component of the gradient.
         * @return \e V the value of the sum.
         *
         * The gradients will only be computed if the CircularEngine object was
         * created with this capability (e.g., via \e gradp = true in
         * SphericalHarmonic::Circle).  If not, \e gradx, etc., will not be
         * touched.  The arguments must satisfy <i>coslon</i><sup>2</sup> +
         * <i>sinlon</i><sup>2</sup> = 1.
         **********************************************************************/
        double LongitudeSum(double coslon, double sinlon,
                        [System::Runtime::InteropServices::Out] double% gradx,
                        [System::Runtime::InteropServices::Out] double% grady,
                        [System::Runtime::InteropServices::Out] double% gradz);

        /**
         * Evaluate the sum and its gradient for a particular longitude.
         *
         * @param[in] lon the longitude (degrees).
         * @param[out] gradx \e x component of the gradient.
         * @param[out] grady \e y component of the gradient.
         * @param[out] gradz \e z component of the gradient.
         * @return \e V the value of the sum.
         *
         * The gradients will only be computed if the CircularEngine object was
         * created with this capability (e.g., via \e gradp = true in
         * SphericalHarmonic::Circle).  If not, \e gradx, etc., will not be
         * touched.
         **********************************************************************/
        double LongitudeSum(double lon,
                    [System::Runtime::InteropServices::Out] double% gradx,
                    [System::Runtime::InteropServices::Out] double% grady,
                    [System::Runtime::InteropServices::Out] double% gradz);
    };
} //namespace NETGeographicLib
