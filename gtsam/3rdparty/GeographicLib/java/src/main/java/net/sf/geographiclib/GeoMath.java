/**
 * Implementation of the net.sf.geographiclib.GeoMath class
 *
 * Copyright (c) Charles Karney (2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/
package net.sf.geographiclib;

/**
 * Mathematical functions needed by GeographicLib.
 * <p>
 * Define mathematical functions and constants so that any version of Java
 * can be used.
 **********************************************************************/
public class GeoMath {
  /**
   * The number of binary digits in the fraction of a double precision
   * number (equivalent to C++'s {@code numeric_limits<double>::digits}).
   **********************************************************************/
  public static final int digits = 53;
  /**
   * Equivalent to C++'s {@code numeric_limits<double>::epsilon()}.  In Java
   * version 1.5 and later, Math.ulp(1.0) can be used.
   **********************************************************************/
  public static final double epsilon = Math.pow(0.5, digits - 1);
  /**
   * Equivalent to C++'s {@code numeric_limits<double>::min()}.  In Java
   * version 1.6 and later, Double.MIN_NORMAL can be used.
   **********************************************************************/
  public static final double min = Math.pow(0.5, 1022);
  /**
   * The number of radians in a degree.  In Java version 1.2 and later,
   * Math.toRadians and Math.toDegrees can be used.
   **********************************************************************/
  public static final double degree = Math.PI / 180;

  /**
   * Square a number.
   * <p>
   * @param x the argument.
   * @return <i>x</i><sup>2</sup>.
   **********************************************************************/
  public static double sq(double x) { return x * x; }

  /**
   * The hypotenuse function avoiding underflow and overflow.  In Java version
   * 1.5 and later, Math.hypot can be used.
   * <p>
   * @param x the first argument.
   * @param y the second argument.
   * @return sqrt(<i>x</i><sup>2</sup> + <i>y</i><sup>2</sup>).
   **********************************************************************/
  public static double hypot(double x, double y) {
    x = Math.abs(x); y = Math.abs(y);
    double a = Math.max(x, y), b = Math.min(x, y) / (a != 0 ? a : 1);
    return a * Math.sqrt(1 + b * b);
    // For an alternative method see
    // C. Moler and D. Morrision (1983) http://dx.doi.org/10.1147/rd.276.0577
    // and A. A. Dubrulle (1983) http://dx.doi.org/10.1147/rd.276.0582
  }

  /**
   * log(1 + <i>x</i>) accurate near <i>x</i> = 0.  In Java version 1.5 and
   * later, Math.log1p can be used.
   * <p>
   * This is taken from D. Goldberg,
   * <a href="http://dx.doi.org/10.1145/103162.103163">What every computer
   * scientist should know about floating-point arithmetic</a> (1991),
   * Theorem 4.  See also, N. J. Higham, Accuracy and Stability of Numerical
   * Algorithms, 2nd Edition (SIAM, 2002), Answer to Problem 1.5, p 528.
   * <p>
   * @param x the argument.
   * @return log(1 + <i>x</i>).
   **********************************************************************/
  public static double log1p(double x) {
    double
      y = 1 + x,
      z = y - 1;
    // Here's the explanation for this magic: y = 1 + z, exactly, and z
    // approx x, thus log(y)/z (which is nearly constant near z = 0) returns
    // a good approximation to the true log(1 + x)/x.  The multiplication x *
    // (log(y)/z) introduces little additional error.
    return z == 0 ? x : x * Math.log(y) / z;
  }

  /**
   * The inverse hyperbolic tangent function.  This is defined in terms of
   * GeoMath.log1p(<i>x</i>) in order to maintain accuracy near <i>x</i> = 0.
   * In addition, the odd parity of the function is enforced.
   * <p>
   * @param x the argument.
   * @return atanh(<i>x</i>).
   **********************************************************************/
  public static double atanh(double x)  {
    double y = Math.abs(x);     // Enforce odd parity
    y = Math.log1p(2 * y/(1 - y))/2;
    return x < 0 ? -y : y;
  }

  /**
   * The cube root function.  In Java version 1.5 and later, Math.cbrt can be
   * used.
   * <p>
   * @param x the argument.
   * @return the real cube root of <i>x</i>.
   **********************************************************************/
  public static double cbrt(double x) {
    double y = Math.pow(Math.abs(x), 1/3.0); // Return the real cube root
    return x < 0 ? -y : y;
  }

  /**
   * The error-free sum of two numbers.
   * <p>
   * @param u the first number in the sum.
   * @param v the second number in the sum.
   * @return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = round(<i>u</i> +
   *   <i>v</i>) and <i>t</i> = <i>u</i> + <i>v</i> - <i>s</i>.
   * <p>
   * See D. E. Knuth, TAOCP, Vol 2, 4.2.2, Theorem B.
   **********************************************************************/
  public static Pair sum(double u, double v) {
    double s = u + v;
    double up = s - v;
    double vpp = s - up;
    up -= u;
    vpp -= v;
    double t = -(up + vpp);
    // u + v =       s      + t
    //       = round(u + v) + t
    return new Pair(s, t);
  }

  /**
   * Normalize an angle (restricted input range).
   * <p>
   * @param x the angle in degrees.
   * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
   * <p>
   * <i>x</i> must lie in [&minus;540&deg;, 540&deg;).
   **********************************************************************/
  public static double AngNormalize(double x)
  { return x >= 180 ? x - 360 : (x < -180 ? x + 360 : x); }

  /**
   * Normalize an arbitrary angle.
   * <p>
   * @param x the angle in degrees.
   * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
   * <p>
   * The range of <i>x</i> is unrestricted.
   **********************************************************************/
  public static double AngNormalize2(double x)
  { return AngNormalize(x % 360.0); }

  /**
   * Difference of two angles reduced to [&minus;180&deg;, 180&deg;]
   * <p>
   * @param x the first angle in degrees.
   * @param y the second angle in degrees.
   * @return <i>y</i> &minus; <i>x</i>, reduced to the range [&minus;180&deg;,
   *   180&deg;].
   * <p>
   * <i>x</i> and <i>y</i> must both lie in [&minus;180&deg;, 180&deg;].  The
   * result is equivalent to computing the difference exactly, reducing it to
   * (&minus;180&deg;, 180&deg;] and rounding the result.  Note that this
   * prescription allows &minus;180&deg; to be returned (e.g., if <i>x</i> is
   * tiny and negative and <i>y</i> = 180&deg;).
   **********************************************************************/
  public static double AngDiff(double x, double y) {
    double d, t;
    { Pair r = sum(-x, y); d = r.first; t = r.second; }
    if ((d - 180.0) + t > 0.0) // y - x > 180
      d -= 360.0;            // exact
    else if ((d + 180.0) + t <= 0.0) // y - x <= -180
      d += 360.0;            // exact
    return d + t;
  }
  /**
   * Test for finiteness.
   * <p>
   * @param x the argument.
   * @return true if number is finite, false if NaN or infinite.
   **********************************************************************/
  public static boolean isfinite(double x) {
    return Math.abs(x) <= Double.MAX_VALUE;
  }

  private GeoMath() {}
}
