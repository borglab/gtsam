#pragma once
/**
 * \file NETGeographicLib/NormalGravity.h
 * \brief Header for NETGeographicLib::NormalGravity class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    ref class Geocentric;
  /**
   * \brief .NET wrapper for GeographicLib::NormalGravity.
   *
   * This class allows .NET applications to access GeographicLib::NormalGravity.
   *
   * "Normal" gravity refers to an idealization of the earth which is modeled
   * as an rotating ellipsoid.  The eccentricity of the ellipsoid, the rotation
   * speed, and the distribution of mass within the ellipsoid are such that the
   * surface of the ellipsoid is a surface of constant potential (gravitational
   * plus centrifugal).  The acceleration due to gravity is therefore
   * perpendicular to the surface of the ellipsoid.
   *
   * There is a closed solution to this problem which is implemented here.
   * Series "approximations" are only used to evaluate certain combinations of
   * elementary functions where use of the closed expression results in a loss
   * of accuracy for small arguments due to cancellation of the two leading
   * terms.  However these series include sufficient terms to give full machine
   * precision.
   *
   * Definitions:
   * - <i>V</i><sub>0</sub>, the gravitational contribution to the normal
   *   potential;
   * - &Phi;, the rotational contribution to the normal potential;
   * - \e U = <i>V</i><sub>0</sub> + &Phi;, the total
   *   potential;
   * - <b>&Gamma;</b> = &nabla;<i>V</i><sub>0</sub>, the acceleration due to
   *   mass of the earth;
   * - <b>f</b> = &nabla;&Phi;, the centrifugal acceleration;
   * - <b>&gamma;</b> = &nabla;\e U = <b>&Gamma;</b> + <b>f</b>, the normal
   *   acceleration;
   * - \e X, \e Y, \e Z, geocentric coordinates;
   * - \e x, \e y, \e z, local cartesian coordinates used to denote the east,
   *   north and up directions.
   *
   * References:
   * - W. A. Heiskanen and H. Moritz, Physical Geodesy (Freeman, San
   *   Francisco, 1967), Secs. 1-19, 2-7, 2-8 (2-9, 2-10), 6-2 (6-3).
   * - H. Moritz, Geodetic Reference System 1980, J. Geodesy 54(3), 395-405
   *   (1980) https://doi.org/10.1007/BF02521480
   *
   * C# Example:
   * \include example-NormalGravity.cs
   * Managed C++ Example:
   * \include example-NormalGravity.cpp
   * Visual Basic Example:
   * \include example-NormalGravity.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A constructor has been provided for creating standard WGS84 and GRS80
   * gravity models.
   *
   * The following functions are implemented as properties:
   * MajorRadius, MassConstant, AngularVelocity, Flattening,
   * EquatorialGravity, PolarGravity, GravityFlattening, SurfacePotential.
   **********************************************************************/
    public ref class NormalGravity
    {
        private:
        // a pointer to the unmanaged GeographicLib::NormalGravity.
        const GeographicLib::NormalGravity* m_pNormalGravity;

        // the finalizer frees the unmanaged memory when the object is destroyed.
        !NormalGravity(void);
    public:
        //! The enumerated standard gravity models.
        enum class StandardModels
        {
            WGS84,  //!< WGS84 gravity model.
            GRS80   //!< GRS80 gravity model.
        };

        /** \name Setting up the normal gravity
         **********************************************************************/
        ///@{
        /**
         * Constructor for the normal gravity.
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] GM mass constant of the ellipsoid
         *   (meters<sup>3</sup>/seconds<sup>2</sup>); this is the product of \e G
         *   the gravitational constant and \e M the mass of the earth (usually
         *   including the mass of the earth's atmosphere).
         * @param[in] omega the angular velocity (rad s<sup>&minus;1</sup>).
         * @param[in] f_J2 either the flattening of the ellipsoid \e f or the
         *   the dynamical form factor \e J2.
         * @param[out] geometricp if true, then \e f_J2 denotes the
         *   flattening, else it denotes the dynamical form factor \e J2.
         * @exception if \e a is not positive or if the other parameters do not
         *   obey the restrictions given below.
         *
         * The shape of the ellipsoid can be given in one of two ways:
         * - geometrically (\e geomtricp = true), the ellipsoid is defined by the
         *   flattening \e f = (\e a &minus; \e b) / \e a, where \e a and \e b are
         *   the equatorial radius and the polar semi-axis.  The parameters should
         *   obey \e a &gt; 0, \e f &lt; 1.  There are no restrictions on \e GM or
         *   \e omega, in particular, \e GM need not be positive.
         * - physically (\e geometricp = false), the ellipsoid is defined by the
         *   dynamical form factor <i>J</i><sub>2</sub> = (\e C &minus; \e A) /
         *   <i>Ma</i><sup>2</sup>, where \e A and \e C are the equatorial and
         *   polar moments of inertia and \e M is the mass of the earth.  The
         *   parameters should obey \e a &gt; 0, \e GM &gt; 0 and \e J2 &lt; 1/3
         *   &minus; (<i>omega</i><sup>2</sup><i>a</i><sup>3</sup>/<i>GM</i>)
         *   8/(45&pi;).  There is no restriction on \e omega.
         **********************************************************************/
        NormalGravity(double a, double GM, double omega, double f_J2,
                      bool geometricp);

        /**
         * A constructor for creating standard gravity models..
         * @param[in] model Specifies the desired model.
         **********************************************************************/
        NormalGravity(StandardModels model);

        /**
         * A constructor that accepts a GeographicLib::NormalGravity.
         * For internal use only.
         * @param g An existing GeographicLib::NormalGravity.
         **********************************************************************/
        NormalGravity( const GeographicLib::NormalGravity& g);
        ///@}

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~NormalGravity()
        { this->!NormalGravity(); }

        /** \name Compute the gravity
         **********************************************************************/
        ///@{
        /**
         * Evaluate the gravity on the surface of the ellipsoid.
         *
         * @param[in] lat the geographic latitude (degrees).
         * @return &gamma; the acceleration due to gravity, positive downwards
         *   (m s<sup>&minus;2</sup>).
         *
         * Due to the axial symmetry of the ellipsoid, the result is independent of
         * the value of the longitude.  This acceleration is perpendicular to the
         * surface of the ellipsoid.  It includes the effects of the earth's
         * rotation.
         **********************************************************************/
        double SurfaceGravity(double lat);

        /**
         * Evaluate the gravity at an arbitrary point above (or below) the
         * ellipsoid.
         *
         * @param[in] lat the geographic latitude (degrees).
         * @param[in] h the height above the ellipsoid (meters).
         * @param[out] gammay the northerly component of the acceleration
         *   (m s<sup>&minus;2</sup>).
         * @param[out] gammaz the upward component of the acceleration
         *   (m s<sup>&minus;2</sup>); this is usually negative.
         * @return \e U the corresponding normal potential.
         *
         * Due to the axial symmetry of the ellipsoid, the result is independent of
         * the value of the longitude and the easterly component of the
         * acceleration vanishes, \e gammax = 0.  The function includes the effects
         * of the earth's rotation.  When \e h = 0, this function gives \e gammay =
         * 0 and the returned value matches that of NormalGravity::SurfaceGravity.
         **********************************************************************/
        double Gravity(double lat, double h,
            [System::Runtime::InteropServices::Out] double% gammay,
            [System::Runtime::InteropServices::Out] double% gammaz);

        /**
         * Evaluate the components of the acceleration due to gravity and the
         * centrifugal acceleration in geocentric coordinates.
         *
         * @param[in] X geocentric coordinate of point (meters).
         * @param[in] Y geocentric coordinate of point (meters).
         * @param[in] Z geocentric coordinate of point (meters).
         * @param[out] gammaX the \e X component of the acceleration
         *   (m s<sup>&minus;2</sup>).
         * @param[out] gammaY the \e Y component of the acceleration
         *   (m s<sup>&minus;2</sup>).
         * @param[out] gammaZ the \e Z component of the acceleration
         *   (m s<sup>&minus;2</sup>).
         * @return \e U = <i>V</i><sub>0</sub> + &Phi; the sum of the
         *   gravitational and centrifugal potentials
         *   (m<sup>2</sup> s<sup>&minus;2</sup>).
         *
         * The acceleration given by <b>&gamma;</b> = &nabla;\e U =
         * &nabla;<i>V</i><sub>0</sub> + &nabla;&Phi; = <b>&Gamma;</b> + <b>f</b>.
         **********************************************************************/
        double U(double X, double Y, double Z,
                     [System::Runtime::InteropServices::Out] double% gammaX,
                     [System::Runtime::InteropServices::Out] double% gammaY,
                     [System::Runtime::InteropServices::Out] double% gammaZ);

        /**
         * Evaluate the components of the acceleration due to gravity alone in
         * geocentric coordinates.
         *
         * @param[in] X geocentric coordinate of point (meters).
         * @param[in] Y geocentric coordinate of point (meters).
         * @param[in] Z geocentric coordinate of point (meters).
         * @param[out] GammaX the \e X component of the acceleration due to gravity
         *   (m s<sup>&minus;2</sup>).
         * @param[out] GammaY the \e Y component of the acceleration due to gravity
         *   (m s<sup>&minus;2</sup>).
         * @param[out] GammaZ the \e Z component of the acceleration due to gravity
         *   (m s<sup>&minus;2</sup>).
         * @return <i>V</i><sub>0</sub> the gravitational potential
         *   (m<sup>2</sup> s<sup>&minus;2</sup>).
         *
         * This function excludes the centrifugal acceleration and is appropriate
         * to use for space applications.  In terrestrial applications, the
         * function NormalGravity::U (which includes this effect) should usually be
         * used.
         **********************************************************************/
        double V0(double X, double Y, double Z,
                      [System::Runtime::InteropServices::Out] double% GammaX,
                      [System::Runtime::InteropServices::Out] double% GammaY,
                      [System::Runtime::InteropServices::Out] double% GammaZ);

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
         *   s<sup>&minus;2</sup>).
         *
         * &Phi; is independent of \e Z, thus \e fZ = 0.  This function
         * NormalGravity::U sums the results of NormalGravity::V0 and
         * NormalGravity::Phi.
         **********************************************************************/
        double Phi(double X, double Y,
            [System::Runtime::InteropServices::Out] double% fX,
            [System::Runtime::InteropServices::Out] double% fY);
        ///@}

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e GM the mass constant of the ellipsoid
         *   (m<sup>3</sup> s<sup>&minus;2</sup>).  This is the value used in the
         *   constructor.
         **********************************************************************/
        property double MassConstant { double get(); }

        /**
         * @return \e J<sub>n</sub> the dynamical form factors of the ellipsoid.
         *
         * If \e n = 2 (the default), this is the value of <i>J</i><sub>2</sub>
         * used in the constructor.  Otherwise it is the zonal coefficient of the
         * Legendre harmonic sum of the normal gravitational potential.  Note that
         * \e J<sub>n</sub> = 0 if \e n is odd.  In most gravity applications,
         * fully normalized Legendre functions are used and the corresponding
         * coefficient is <i>C</i><sub><i>n</i>0</sub> = &minus;\e J<sub>n</sub> /
         * sqrt(2 \e n + 1).
         **********************************************************************/
        double DynamicalFormFactor(int n);

        /**
         * @return &omega; the angular velocity of the ellipsoid (rad
         *   s<sup>&minus;1</sup>).  This is the value used in the constructor.
         **********************************************************************/
        property double AngularVelocity { double get(); }

        /**
         * @return <i>f</i> the flattening of the ellipsoid (\e a &minus; \e b)/\e
         *   a.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * @return &gamma;<sub>e</sub> the normal gravity at equator (m
         *   s<sup>&minus;2</sup>).
         **********************************************************************/
        property double EquatorialGravity { double get(); }

        /**
         * @return &gamma;<sub>p</sub> the normal gravity at poles (m
         *   s<sup>&minus;2</sup>).
         **********************************************************************/
        property double PolarGravity { double get(); }

        /**
         * @return <i>f*</i> the gravity flattening (&gamma;<sub>p</sub> &minus;
         *   &gamma;<sub>e</sub>) / &gamma;<sub>e</sub>.
         **********************************************************************/
        property double GravityFlattening { double get(); }

        /**
         * @return <i>U</i><sub>0</sub> the constant normal potential for the
         *   surface of the ellipsoid (m<sup>2</sup> s<sup>&minus;2</sup>).
         **********************************************************************/
        property double SurfacePotential { double get(); }

        /**
         * @return the Geocentric object used by this instance.
         **********************************************************************/
        Geocentric^ Earth();
        ///@}

        /**
         * A global instantiation of NormalGravity for the WGS84 ellipsoid.
        **********************************************************************/
        static NormalGravity^ WGS84();

        /**
         * A global instantiation of NormalGravity for the GRS80 ellipsoid.
        **********************************************************************/
        static NormalGravity^ GRS80();

        /**
         * Compute the flattening from the dynamical form factor.
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] GM mass constant of the ellipsoid
         *   (meters<sup>3</sup>/seconds<sup>2</sup>); this is the product of \e G
         *   the gravitational constant and \e M the mass of the earth (usually
         *   including the mass of the earth's atmosphere).
         * @param[in] omega the angular velocity (rad s<sup>&minus;1</sup>).
         * @param[in] J2 the dynamical form factor.
         * @return \e f the flattening of the ellipsoid.
         **********************************************************************/
        static double J2ToFlattening(double a, double GM, double omega,
                                     double J2);

        /**
         * Compute the dynamical form factor from the flattening.
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] GM mass constant of the ellipsoid
         *   (meters<sup>3</sup>/seconds<sup>2</sup>); this is the product of \e G
         *   the gravitational constant and \e M the mass of the earth (usually
         *   including the mass of the earth's atmosphere).
         * @param[in] omega the angular velocity (rad s<sup>&minus;1</sup>).
         * @param[in] f the flattening of the ellipsoid.
         * @return \e J2 the dynamical form factor.
         **********************************************************************/
        static double FlatteningToJ2(double a, double GM, double omega,
                                     double f);
    };
} //namespace NETGeographicLib
