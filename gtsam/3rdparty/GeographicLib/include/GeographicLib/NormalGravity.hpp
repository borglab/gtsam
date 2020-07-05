/**
 * \file NormalGravity.hpp
 * \brief Header for GeographicLib::NormalGravity class
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
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
   * ellipsoid is a "level ellipoid", a surface of constant potential
   * (gravitational plus centrifugal).  The acceleration due to gravity is
   * therefore perpendicular to the surface of the ellipsoid.
   *
   * Because the distribution of mass within the ellipsoid is unspecified, only
   * the potential exterior to the ellipsoid is well defined.  In this class,
   * the mass is assumed to be to concentrated on a "focal disc" of radius,
   * (<i>a</i><sup>2</sup> &minus; <i>b</i><sup>2</sup>)<sup>1/2</sup>, where
   * \e a is the equatorial radius of the ellipsoid and \e b is its polar
   * semi-axis.  In the case of an oblate ellipsoid, the mass is concentrated
   * on a "focal rod" of length 2(<i>b</i><sup>2</sup> &minus;
   * <i>a</i><sup>2</sup>)<sup>1/2</sup>.  As a result the potential is well
   * defined everywhere.
   *
   * There is a closed solution to this problem which is implemented here.
   * Series "approximations" are only used to evaluate certain combinations of
   * elementary functions where use of the closed expression results in a loss
   * of accuracy for small arguments due to cancellation of the leading terms.
   * However these series include sufficient terms to give full machine
   * precision.
   *
   * Although the formulation used in this class applies to ellipsoids with
   * arbitrary flattening, in practice, its use should be limited to about
   * <i>b</i>/\e a &isin; [0.01, 100] or \e f &isin; [&minus;99, 0.99].
   *
   * Definitions:
   * - <i>V</i><sub>0</sub>, the gravitational contribution to the normal
   *   potential;
   * - &Phi;, the rotational contribution to the normal potential;
   * - \e U = <i>V</i><sub>0</sub> + &Phi;, the total potential;
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
   * - C. Somigliana, Teoria generale del campo gravitazionale dell'ellissoide
   *   di rotazione, Mem. Soc. Astron. Ital, <b>4</b>, 541--599 (1929).
   * - W. A. Heiskanen and H. Moritz, Physical Geodesy (Freeman, San
   *   Francisco, 1967), Secs. 1-19, 2-7, 2-8 (2-9, 2-10), 6-2 (6-3).
   * - B. Hofmann-Wellenhof, H. Moritz, Physical Geodesy (Second edition,
   *   Springer, 2006) https://doi.org/10.1007/978-3-211-33545-1
   * - H. Moritz, Geodetic Reference System 1980, J. Geodesy 54(3), 395-405
   *   (1980) https://doi.org/10.1007/BF02521480
   *
   * For more information on normal gravity see \ref normalgravity.
   *
   * Example of use:
   * \include example-NormalGravity.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT NormalGravity {
  private:
    static const int maxit_ = 20;
    typedef Math::real real;
    friend class GravityModel;
    real _a, _GM, _omega, _f, _J2, _omega2, _aomega2;
    real _e2, _ep2, _b, _E, _U0, _gammae, _gammap, _Q0, _k, _fstar;
    Geocentric _earth;
    static real atanzz(real x, bool alt) {
      // This routine obeys the identity
      //   atanzz(x, alt) = atanzz(-x/(1+x), !alt)
      //
      // Require x >= -1.  Best to call with alt, s.t. x >= 0; this results in
      // a call to atan, instead of asin, or to asinh, instead of atanh.
      using std::sqrt; using std::abs; using std::atan; using std::asin;
      real z = sqrt(abs(x));
      return x == 0 ? 1 :
        (alt ?
         (!(x < 0) ? Math::asinh(z) : asin(z)) / sqrt(abs(x) / (1 + x)) :
         (!(x < 0) ? atan(z) : Math::atanh(z)) / z);
    }
    static real atan7series(real x);
    static real atan5series(real x);
    static real Qf(real x, bool alt);
    static real Hf(real x, bool alt);
    static real QH3f(real x, bool alt);
    real Jn(int n) const;
    void Initialize(real a, real GM, real omega, real f_J2, bool geometricp);
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
     * @param[in] f_J2 either the flattening of the ellipsoid \e f or the
     *   the dynamical form factor \e J2.
     * @param[out] geometricp if true (the default), then \e f_J2 denotes the
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
    NormalGravity(real a, real GM, real omega, real f_J2,
                  bool geometricp = true);
    /**
     * \deprecated Old constructor for the normal gravity.
     *
     * @param[in] a equatorial radius (meters).
     * @param[in] GM mass constant of the ellipsoid
     *   (meters<sup>3</sup>/seconds<sup>2</sup>); this is the product of \e G
     *   the gravitational constant and \e M the mass of the earth (usually
     *   including the mass of the earth's atmosphere).
     * @param[in] omega the angular velocity (rad s<sup>&minus;1</sup>).
     * @param[in] f the flattening of the ellipsoid.
     * @param[in] J2 the dynamical form factor.
     * @exception if \e a is not positive or the other constants are
     *   inconsistent (see below).
     *
     * If \e omega is non-zero, then exactly one of \e f and \e J2 should be
     * positive and this will be used to define the ellipsoid.  The shape of
     * the ellipsoid can be given in one of two ways:
     * - geometrically, the ellipsoid is defined by the flattening \e f = (\e a
     *   &minus; \e b) / \e a, where \e a and \e b are the equatorial radius
     *   and the polar semi-axis.
     * - physically, the ellipsoid is defined by the dynamical form factor
     *   <i>J</i><sub>2</sub> = (\e C &minus; \e A) / <i>Ma</i><sup>2</sup>,
     *   where \e A and \e C are the equatorial and polar moments of inertia
     *   and \e M is the mass of the earth.
     * .
     * If \e omega, \e f, and \e J2 are all zero, then the ellipsoid becomes a
     * sphere.
     **********************************************************************/
    GEOGRAPHICLIB_DEPRECATED("Use new NormalGravity constructor")
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
    Math::real SurfaceGravity(real lat) const;

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
     * @return \e U the corresponding normal potential
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     *
     * Due to the axial symmetry of the ellipsoid, the result is independent of
     * the value of the longitude and the easterly component of the
     * acceleration vanishes, \e gammax = 0.  The function includes the effects
     * of the earth's rotation.  When \e h = 0, this function gives \e gammay =
     * 0 and the returned value matches that of NormalGravity::SurfaceGravity.
     **********************************************************************/
    Math::real Gravity(real lat, real h, real& gammay, real& gammaz)
      const;

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
                 real& gammaX, real& gammaY, real& gammaZ) const;

    /**
     * Evaluate the components of the acceleration due to the gravitational
     * force in geocentric coordinates.
     *
     * @param[in] X geocentric coordinate of point (meters).
     * @param[in] Y geocentric coordinate of point (meters).
     * @param[in] Z geocentric coordinate of point (meters).
     * @param[out] GammaX the \e X component of the acceleration due to the
     *   gravitational force (m s<sup>&minus;2</sup>).
     * @param[out] GammaY the \e Y component of the acceleration due to the
     * @param[out] GammaZ the \e Z component of the acceleration due to the
     *   gravitational force (m s<sup>&minus;2</sup>).
     * @return <i>V</i><sub>0</sub> the gravitational potential
     *   (m<sup>2</sup> s<sup>&minus;2</sup>).
     *
     * This function excludes the centrifugal acceleration and is appropriate
     * to use for space applications.  In terrestrial applications, the
     * function NormalGravity::U (which includes this effect) should usually be
     * used.
     **********************************************************************/
    Math::real V0(real X, real Y, real Z,
                  real& GammaX, real& GammaY, real& GammaZ) const;

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
    Math::real Phi(real X, real Y, real& fX, real& fY) const;
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
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const
    { return Init() ? _a : Math::NaN(); }

    /**
     * @return \e GM the mass constant of the ellipsoid
     *   (m<sup>3</sup> s<sup>&minus;2</sup>).  This is the value used in the
     *   constructor.
     **********************************************************************/
    Math::real MassConstant() const
    { return Init() ? _GM : Math::NaN(); }

    /**
     * @return <i>J</i><sub><i>n</i></sub> the dynamical form factors of the
     *   ellipsoid.
     *
     * If \e n = 2 (the default), this is the value of <i>J</i><sub>2</sub>
     * used in the constructor.  Otherwise it is the zonal coefficient of the
     * Legendre harmonic sum of the normal gravitational potential.  Note that
     * <i>J</i><sub><i>n</i></sub> = 0 if \e n is odd.  In most gravity
     * applications, fully normalized Legendre functions are used and the
     * corresponding coefficient is <i>C</i><sub><i>n</i>0</sub> =
     * &minus;<i>J</i><sub><i>n</i></sub> / sqrt(2 \e n + 1).
     **********************************************************************/
    Math::real DynamicalFormFactor(int n = 2) const
    { return Init() ? ( n == 2 ? _J2 : Jn(n)) : Math::NaN(); }

    /**
     * @return &omega; the angular velocity of the ellipsoid (rad
     *   s<sup>&minus;1</sup>).  This is the value used in the constructor.
     **********************************************************************/
    Math::real AngularVelocity() const
    { return Init() ? _omega : Math::NaN(); }

    /**
     * @return <i>f</i> the flattening of the ellipsoid (\e a &minus; \e b)/\e
     *   a.
     **********************************************************************/
    Math::real Flattening() const
    { return Init() ? _f : Math::NaN(); }

    /**
     * @return &gamma;<sub>e</sub> the normal gravity at equator (m
     *   s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real EquatorialGravity() const
    { return Init() ? _gammae : Math::NaN(); }

    /**
     * @return &gamma;<sub>p</sub> the normal gravity at poles (m
     *   s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real PolarGravity() const
    { return Init() ? _gammap : Math::NaN(); }

    /**
     * @return <i>f*</i> the gravity flattening (&gamma;<sub>p</sub> &minus;
     *   &gamma;<sub>e</sub>) / &gamma;<sub>e</sub>.
     **********************************************************************/
    Math::real GravityFlattening() const
    { return Init() ? _fstar : Math::NaN(); }

    /**
     * @return <i>U</i><sub>0</sub> the constant normal potential for the
     *   surface of the ellipsoid (m<sup>2</sup> s<sup>&minus;2</sup>).
     **********************************************************************/
    Math::real SurfacePotential() const
    { return Init() ? _U0 : Math::NaN(); }

    /**
     * @return the Geocentric object used by this instance.
     **********************************************************************/
    const Geocentric& Earth() const { return _earth; }
    ///@}

    /**
     * A global instantiation of NormalGravity for the WGS84 ellipsoid.
     **********************************************************************/
    static const NormalGravity& WGS84();

    /**
     * A global instantiation of NormalGravity for the GRS80 ellipsoid.
     **********************************************************************/
    static const NormalGravity& GRS80();

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
     *
     * This routine requires \e a &gt; 0, \e GM &gt; 0, \e J2 &lt; 1/3 &minus;
     * <i>omega</i><sup>2</sup><i>a</i><sup>3</sup>/<i>GM</i> 8/(45&pi;).  A
     * NaN is returned if these conditions do not hold.  The restriction to
     * positive \e GM is made because for negative \e GM two solutions are
     * possible.
     **********************************************************************/
    static Math::real J2ToFlattening(real a, real GM, real omega, real J2);

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
     *
     * This routine requires \e a &gt; 0, \e GM &ne; 0, \e f &lt; 1.  The
     * values of these parameters are not checked.
     **********************************************************************/
    static Math::real FlatteningToJ2(real a, real GM, real omega, real f);
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_NORMALGRAVITY_HPP
