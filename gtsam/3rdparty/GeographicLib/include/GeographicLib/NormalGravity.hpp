/**
 * \file NormalGravity.hpp
 * \brief Header for GeographicLib::NormalGravity class
 *
 * Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_NORMALGRAVITY_HPP)
#define GEOGRAPHICLIB_NORMALGRAVITY_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geocentric.hpp>

namespace GeographicLib {

  /**
   * \brief The normal gravity of the earth
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
   *   (1980) http://dx.doi.org/10.1007/BF02521480
   *
   * Example of use:
   * \include example-NormalGravity.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT NormalGravity {
  private:
    static const int maxit_ = 10;
    typedef Math::real real;
    friend class GravityModel;
    real _a, _GM, _omega, _f, _J2, _omega2, _aomega2;
    real _e2, _ep2, _b, _E, _U0, _gammae, _gammap, _q0, _m, _k, _fstar;
    Geocentric _earth;
    static Math::real qf(real ep2) throw();
    static Math::real qpf(real ep2) throw();
    Math::real Jn(int n) const throw();
  public:

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
     * @param[in] f the flattening of the ellipsoid.
     * @param[in] J2 dynamical form factor.
     * @exception if \e a is not positive or the other constants are
     *   inconsistent (see below).
     *
     * Exactly one of \e f and \e J2 should be positive and this will be used
     * to define the ellipsoid.  The shape of the ellipsoid can be given in one
     * of two ways:
     * - geometrically, the ellipsoid is defined by the flattening \e f = (\e a
     *   &minus; \e b) / \e a, where \e a and \e b are the equatorial radius
     *   and the polar semi-axis.
     * - physically, the ellipsoid is defined by the dynamical form factor
     *   <i>J</i><sub>2</sub> = (\e C &minus; \e A) / <i>Ma</i><sup>2</sup>,
     *   where \e A and \e C are the equatorial and polar moments of inertia
     *   and \e M is the mass of the earth.
     **********************************************************************/
    NormalGravity(real a, real GM, real omega, real f, real J2);

    /**
     * A default constructor for the normal gravity.  This sets up an
     * uninitialized object and is used by GravityModel which constructs this
     * object before it has read in the parameters for the reference ellipsoid.
     **********************************************************************/
    NormalGravity() : _a(-1) {}
    ///@}

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
    Math::real SurfaceGravity(real lat) const throw();

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
    Math::real Gravity(real lat, real h, real& gammay, real& gammaz)
      const throw();

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
    Math::real U(real X, real Y, real Z,
                 real& gammaX, real& gammaY, real& gammaZ) const throw();

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
    Math::real V0(real X, real Y, real Z,
                  real& GammaX, real& GammaY, real& GammaZ) const throw();

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
    Math::real Phi(real X, real Y, real& fX, real& fY) const throw();
    ///@}

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return true if the object has been initialized.
     **********************************************************************/
    bool Init() const throw() { return _a > 0; }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const throw()
    { return Init() ? _a : Math::NaN<real>(); }

    /**
     * @return \e GM the mass constant of the ellipsoid
     *   (m<sup>3</sup> s<sup>&minus;2</sup>).  This is the value used in the
     *   constructor.
     **********************************************************************/
    Math::real MassConstant() const throw()
    { return Init() ? _GM : Math::NaN<real>(); }

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
    Math::real DynamicalFormFactor(int n = 2) const throw()
    { return Init() ? ( n == 2 ? _J2 : Jn(n)) : Math::NaN<real>(); }

    /**
     * @return &omega; the angular velocity of the ellipsoid (rad
     *   s<sup>&minus;1</sup>).  This is the value used in the constructor.
     **********************************************************************/
    Math::real AngularVelocity() const throw()
    { return Init() ? _omega : Math::NaN<real>(); }

    /**
     * @return <i>f</i> the flattening of the ellipsoid (\e a &minus; \e b)/\e
     *   a.
     **********************************************************************/
    Math::real Flattening() const throw()
    { return Init() ? _f : Math::NaN<real>(); }

    /**
     * @return &gamma;<sub>e</sub> the normal gravity at equator (m
     *   s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real EquatorialGravity() const throw()
    { return Init() ? _gammae : Math::NaN<real>(); }

    /**
     * @return &gamma;<sub>p</sub> the normal gravity at poles (m
     *   s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real PolarGravity() const throw()
    { return Init() ? _gammap : Math::NaN<real>(); }

    /**
     * @return <i>f*</i> the gravity flattening (&gamma;<sub>p</sub> &minus;
     *   &gamma;<sub>e</sub>) / &gamma;<sub>e</sub>.
     **********************************************************************/
    Math::real GravityFlattening() const throw()
    { return Init() ? _fstar : Math::NaN<real>(); }

    /**
     * @return <i>U</i><sub>0</sub> the constant normal potential for the
     *   surface of the ellipsoid (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real SurfacePotential() const throw()
    { return Init() ? _U0 : Math::NaN<real>(); }

    /**
     * @return the Geocentric object used by this instance.
     **********************************************************************/
    const Geocentric& Earth() const throw() { return _earth; }
    ///@}

    /**
     * A global instantiation of NormalGravity for the WGS84 ellipsoid.
     **********************************************************************/
    static const NormalGravity WGS84;

    /**
     * A global instantiation of NormalGravity for the GRS80 ellipsoid.
     **********************************************************************/
    static const NormalGravity GRS80;
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_NORMALGRAVITY_HPP
