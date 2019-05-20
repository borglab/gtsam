/**
 * Implementation of the net.sf.geographiclib.GeoMath class
 *
 * Copyright (c) Charles Karney (2013-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
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
    // C. Moler and D. Morrision (1983) https://doi.org/10.1147/rd.276.0577
    // and A. A. Dubrulle (1983) https://doi.org/10.1147/rd.276.0582
  }

  /**
   * log(1 + <i>x</i>) accurate near <i>x</i> = 0.  In Java version 1.5 and
   * later, Math.log1p can be used.
   * <p>
   * This is taken from D. Goldberg,
   * <a href="https://doi.org/10.1145/103162.103163">What every computer
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
   * Copy the sign.  In Java version 1.6 and later, Math.copysign can be used.
   * <p>
   * @param x gives the magitude of the result.
   * @param y gives the sign of the result.
   * @return value with the magnitude of <i>x</i> and with the sign of
   *   <i>y</i>.
   **********************************************************************/
  public static double copysign(double x, double y) {
    return Math.abs(x) * (y < 0 || (y == 0 && 1/y < 0) ? -1 : 1);
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

  public static Pair norm(double sinx, double cosx) {
    double r = hypot(sinx, cosx);
    return new Pair(sinx/r, cosx/r);
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
   * Evaluate a polynomial.
   * <p>
   * @param N the order of the polynomial.
   * @param p the coefficient array (of size <i>N</i> + <i>s</i> + 1 or more).
   * @param s starting index for the array.
   * @param x the variable.
   * @return the value of the polynomial.
   *
   * Evaluate <i>y</i> = &sum;<sub><i>n</i>=0..<i>N</i></sub>
   * <i>p</i><sub><i>s</i>+<i>n</i></sub>
   * <i>x</i><sup><i>N</i>&minus;<i>n</i></sup>.  Return 0 if <i>N</i> &lt; 0.
   * Return <i>p</i><sub><i>s</i></sub>, if <i>N</i> = 0 (even if <i>x</i> is
   * infinite or a nan).  The evaluation uses Horner's method.
   **********************************************************************/
  public static double polyval(int N, double p[], int s, double x) {
    double y = N < 0 ? 0 : p[s++];
    while (--N >= 0) y = y * x + p[s++];
    return y;
  }

  public static double AngRound(double x) {
    // The makes the smallest gap in x = 1/16 - nextafter(1/16, 0) = 1/2^57
    // for reals = 0.7 pm on the earth if x is an angle in degrees.  (This
    // is about 1000 times more resolution than we get with angles around 90
    // degrees.)  We use this to avoid having to deal with near singular
    // cases when x is non-zero but tiny (e.g., 1.0e-200).  This converts -0 to
    // +0; however tiny negative numbers get converted to -0.
    final double z = 1/16.0;
    if (x == 0) return 0;
    double y = Math.abs(x);
    // The compiler mustn't "simplify" z - (z - y) to y
    y = y < z ? z - (z - y) : y;
    return x < 0 ? -y : y;
  }

  /**
   * Normalize an angle (restricted input range).
   * <p>
   * @param x the angle in degrees.
   * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
   * <p>
   * The range of <i>x</i> is unrestricted.
   **********************************************************************/
  public static double AngNormalize(double x) {
    x = x % 360.0;
    return x <= -180 ? x + 360 : (x <= 180 ? x : x - 360);
  }

  /**
   * Normalize a latitude.
   * <p>
   * @param x the angle in degrees.
   * @return x if it is in the range [&minus;90&deg;, 90&deg;], otherwise
   *   return NaN.
   **********************************************************************/
  public static double LatFix(double x) {
    return Math.abs(x) > 90 ? Double.NaN : x;
  }

  /**
   * The exact difference of two angles reduced to (&minus;180&deg;, 180&deg;].
   * <p>
   * @param x the first angle in degrees.
   * @param y the second angle in degrees.
   * @return Pair(<i>d</i>, <i>e</i>) with <i>d</i> being the rounded
   *   difference and <i>e</i> being the error.
   * <p>
   * The computes <i>z</i> = <i>y</i> &minus; <i>x</i> exactly, reduced to
   * (&minus;180&deg;, 180&deg;]; and then sets <i>z</i> = <i>d</i> + <i>e</i>
   * where <i>d</i> is the nearest representable number to <i>z</i> and
   * <i>e</i> is the truncation error.  If <i>d</i> = &minus;180, then <i>e</i>
   * &gt; 0; If <i>d</i> = 180, then <i>e</i> &le; 0.
   **********************************************************************/
  public static Pair AngDiff(double x, double y) {
    double d, t;
    {
      Pair r = sum(AngNormalize(-x), AngNormalize(y));
      d = AngNormalize(r.first); t = r.second;
    }
    return sum(d == 180 && t > 0 ? -180 : d, t);
  }

  /**
   * Evaluate the sine and cosine function with the argument in degrees
   *
   * @param x in degrees.
   * @return Pair(<i>s</i>, <i>t</i>) with <i>s</i> = sin(<i>x</i>) and
   *   <i>c</i> = cos(<i>x</i>).
   *
   * The results obey exactly the elementary properties of the trigonometric
   * functions, e.g., sin 9&deg; = cos 81&deg; = &minus; sin 123456789&deg;.
   **********************************************************************/
  public static Pair sincosd(double x) {
    // In order to minimize round-off errors, this function exactly reduces
    // the argument to the range [-45, 45] before converting it to radians.
    double r; int q;
    r = x % 360.0;
    q = (int)Math.floor(r / 90 + 0.5);
    r -= 90 * q;
    // now abs(r) <= 45
    r = Math.toRadians(r);
    // Possibly could call the gnu extension sincos
    double s = Math.sin(r), c = Math.cos(r);
    double sinx, cosx;
    switch (q & 3) {
    case  0: sinx =  s; cosx =  c; break;
    case  1: sinx =  c; cosx = -s; break;
    case  2: sinx = -s; cosx = -c; break;
    default: sinx = -c; cosx =  s; break; // case 3
    }
    if (x != 0) { sinx += 0.0; cosx += 0.0; }
    return new Pair(sinx, cosx);
  }

  /**
   * Evaluate the atan2 function with the result in degrees
   *
   * @param y the sine of the angle
   * @param x the cosine of the angle
   * @return atan2(<i>y</i>, <i>x</i>) in degrees.
   *
   * The result is in the range (&minus;180&deg; 180&deg;].  N.B.,
   * atan2d(&plusmn;0, &minus;1) = +180&deg;; atan2d(&minus;&epsilon;,
   * &minus;1) = &minus;180&deg;, for &epsilon; positive and tiny;
   * atan2d(&plusmn;0, 1) = &plusmn;0&deg;.
   **********************************************************************/
  public static double atan2d(double y, double x) {
    // In order to minimize round-off errors, this function rearranges the
    // arguments so that result of atan2 is in the range [-pi/4, pi/4] before
    // converting it to degrees and mapping the result to the correct
    // quadrant.
    int q = 0;
    if (Math.abs(y) > Math.abs(x)) { double t; t = x; x = y; y = t; q = 2; }
    if (x < 0) { x = -x; ++q; }
    // here x >= 0 and x >= abs(y), so angle is in [-pi/4, pi/4]
    double ang = Math.toDegrees(Math.atan2(y, x));
    switch (q) {
      // Note that atan2d(-0.0, 1.0) will return -0.  However, we expect that
      // atan2d will not be called with y = -0.  If need be, include
      //
      //   case 0: ang = 0 + ang; break;
      //
      // and handle mpfr as in AngRound.
    case 1: ang = (y >= 0 ? 180 : -180) - ang; break;
    case 2: ang =  90 - ang; break;
    case 3: ang = -90 + ang; break;
    }
    return ang;
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
