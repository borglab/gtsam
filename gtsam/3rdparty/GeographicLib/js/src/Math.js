/*
 * Math.js
 * Transcription of Math.hpp, Constants.hpp, and Accumulator.hpp into
 * JavaScript.
 *
 * Copyright (c) Charles Karney (2011-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 */

/**
 * @namespace GeographicLib
 * @description The parent namespace for the following modules:
 * - {@link module:GeographicLib/Geodesic GeographicLib/Geodesic} The main
 *   engine for solving geodesic problems via the
 *   {@link module:GeographicLib/Geodesic.Geodesic Geodesic} class.
 * - {@link module:GeographicLib/GeodesicLine GeographicLib/GeodesicLine}
 *   computes points along a single geodesic line via the
 *   {@link module:GeographicLib/GeodesicLine.GeodesicLine GeodesicLine}
 *   class.
 * - {@link module:GeographicLib/PolygonArea GeographicLib/PolygonArea}
 *   computes the area of a geodesic polygon via the
 *   {@link module:GeographicLib/PolygonArea.PolygonArea PolygonArea}
 *   class.
 * - {@link module:GeographicLib/DMS GeographicLib/DMS} handles the decoding
 *   and encoding of angles in degree, minutes, and seconds, via static
 *   functions in this module.
 * - {@link module:GeographicLib/Constants GeographicLib/Constants} defines
 *   constants specifying the version numbers and the parameters for the WGS84
 *   ellipsoid.
 *
 * The following modules are used internally by the package:
 * - {@link module:GeographicLib/Math GeographicLib/Math} defines various
 *   mathematical functions.
 * - {@link module:GeographicLib/Accumulator GeographicLib/Accumulator}
 *   interally used by
 *   {@link module:GeographicLib/PolygonArea.PolygonArea PolygonArea} (via the
 *   {@link module:GeographicLib/Accumulator.Accumulator Accumulator} class)
 *   for summing the contributions to the area of a polygon.
 */
"use strict";
var GeographicLib = {};
GeographicLib.Constants = {};
GeographicLib.Math = {};
GeographicLib.Accumulator = {};

(function(
  /**
   * @exports GeographicLib/Constants
   * @description Define constants defining the version and WGS84 parameters.
   */
  c) {

  /**
   * @constant
   * @summary WGS84 parameters.
   * @property {number} a the equatorial radius (meters).
   * @property {number} f the flattening.
   */
  c.WGS84 = { a: 6378137, f: 1/298.257223563 };
  /**
   * @constant
   * @summary an array of version numbers.
   * @property {number} major the major version number.
   * @property {number} minor the minor version number.
   * @property {number} patch the patch number.
   */
  c.version = { major: 1, minor: 49, patch: 0 };
  /**
   * @constant
   * @summary version string
   */
  c.version_string = "1.49";
})(GeographicLib.Constants);

(function(
  /**
   * @exports GeographicLib/Math
   * @description Some useful mathematical constants and functions (mainly for
   *   internal use).
   */
  m) {

  /**
   * @summary The number of digits of precision in floating-point numbers.
   * @constant {number}
   */
  m.digits = 53;
  /**
   * @summary The machine epsilon.
   * @constant {number}
   */
  m.epsilon = Math.pow(0.5, m.digits - 1);
  /**
   * @summary The factor to convert degrees to radians.
   * @constant {number}
   */
  m.degree = Math.PI/180;

  /**
   * @summary Square a number.
   * @param {number} x the number.
   * @returns {number} the square.
   */
  m.sq = function(x) { return x * x; };

  /**
   * @summary The hypotenuse function.
   * @param {number} x the first side.
   * @param {number} y the second side.
   * @returns {number} the hypotenuse.
   */
  m.hypot = function(x, y) {
    var a, b;
    x = Math.abs(x);
    y = Math.abs(y);
    a = Math.max(x, y); b = Math.min(x, y) / (a ? a : 1);
    return a * Math.sqrt(1 + b * b);
  };

  /**
   * @summary Cube root function.
   * @param {number} x the argument.
   * @returns {number} the real cube root.
   */
  m.cbrt = function(x) {
    var y = Math.pow(Math.abs(x), 1/3);
    return x < 0 ? -y : y;
  };

  /**
   * @summary The log1p function.
   * @param {number} x the argument.
   * @returns {number} log(1 + x).
   */
  m.log1p = function(x) {
    var y = 1 + x,
        z = y - 1;
    // Here's the explanation for this magic: y = 1 + z, exactly, and z
    // approx x, thus log(y)/z (which is nearly constant near z = 0) returns
    // a good approximation to the true log(1 + x)/x.  The multiplication x *
    // (log(y)/z) introduces little additional error.
    return z === 0 ? x : x * Math.log(y) / z;
  };

  /**
   * @summary Inverse hyperbolic tangent.
   * @param {number} x the argument.
   * @returns {number} tanh<sup>&minus;1</sup> x.
   */
  m.atanh = function(x) {
    var y = Math.abs(x);          // Enforce odd parity
    y = m.log1p(2 * y/(1 - y))/2;
    return x < 0 ? -y : y;
  };

  /**
   * @summary Copy the sign.
   * @param {number} x gives the magitude of the result.
   * @param {number} y gives the sign of the result.
   * @returns {number} value with the magnitude of x and with the sign of y.
   */
  m.copysign = function(x, y) {
    return Math.abs(x) * (y < 0 || (y === 0 && 1/y < 0) ? -1 : 1);
  };

  /**
   * @summary An error-free sum.
   * @param {number} u
   * @param {number} v
   * @returns {object} sum with sum.s = round(u + v) and sum.t is u + v &minus;
   *   round(u + v)
   */
  m.sum = function(u, v) {
    var s = u + v,
        up = s - v,
        vpp = s - up,
        t;
    up -= u;
    vpp -= v;
    t = -(up + vpp);
    // u + v =       s      + t
    //       = round(u + v) + t
    return {s: s, t: t};
  };

  /**
   * @summary Evaluate a polynomial.
   * @param {integer} N the order of the polynomial.
   * @param {array} p the coefficient array (of size N + 1) (leading
   *   order coefficient first)
   * @param {number} x the variable.
   * @returns {number} the value of the polynomial.
   */
  m.polyval = function(N, p, s, x) {
    var y = N < 0 ? 0 : p[s++];
    while (--N >= 0) y = y * x + p[s++];
    return y;
  };

  /**
   * @summary Coarsen a value close to zero.
   * @param {number} x
   * @returns {number} the coarsened value.
   */
  m.AngRound = function(x) {
    // The makes the smallest gap in x = 1/16 - nextafter(1/16, 0) = 1/2^57 for
    // reals = 0.7 pm on the earth if x is an angle in degrees.  (This is about
    // 1000 times more resolution than we get with angles around 90 degrees.)
    // We use this to avoid having to deal with near singular cases when x is
    // non-zero but tiny (e.g., 1.0e-200).  This converts -0 to +0; however
    // tiny negative numbers get converted to -0.
    if (x === 0) return x;
    var z = 1/16,
        y = Math.abs(x);
    // The compiler mustn't "simplify" z - (z - y) to y
    y = y < z ? z - (z - y) : y;
    return x < 0 ? -y : y;
  };

  /**
   * @summary Normalize an angle.
   * @param {number} x the angle in degrees.
   * @returns {number} the angle reduced to the range (&minus;180&deg;,
   *   180&deg;].
   */
  m.AngNormalize = function(x) {
    // Place angle in [-180, 180).
    x = x % 360;
    return x <= -180 ? x + 360 : (x <= 180 ? x : x - 360);
  };

  /**
   * @summary Normalize a latitude.
   * @param {number} x the angle in degrees.
   * @returns {number} x if it is in the range [&minus;90&deg;, 90&deg;],
   *   otherwise return NaN.
   */
  m.LatFix = function(x) {
    // Replace angle with NaN if outside [-90, 90].
    return Math.abs(x) > 90 ? Number.NaN : x;
  };

  /**
   * @summary The exact difference of two angles reduced to (&minus;180&deg;,
   *   180&deg;]
   * @param {number} x the first angle in degrees.
   * @param {number} y the second angle in degrees.
   * @return {object} diff the exact difference, y &minus; x.
   *
   * This computes z = y &minus; x exactly, reduced to (&minus;180&deg;,
   * 180&deg;]; and then sets diff.s = d = round(z) and diff.t = e = z &minus;
   * round(z).  If d = &minus;180, then e &gt; 0; If d = 180, then e &le; 0.
   */
  m.AngDiff = function(x, y) {
    // Compute y - x and reduce to [-180,180] accurately.
    var r = m.sum(m.AngNormalize(-x), m.AngNormalize(y)),
        d = m.AngNormalize(r.s),
        t = r.t;
    return m.sum(d === 180 && t > 0 ? -180 : d, t);
  };

  /**
   * @summary Evaluate the sine and cosine function with the argument in
   *   degrees
   * @param {number} x in degrees.
   * @returns {object} r with r.s = sin(x) and r.c = cos(x).
   */
  m.sincosd = function(x) {
    // In order to minimize round-off errors, this function exactly reduces
    // the argument to the range [-45, 45] before converting it to radians.
    var r, q, s, c, sinx, cosx;
    r = x % 360;
    q = Math.floor(r / 90 + 0.5);
    r -= 90 * q;
    // now abs(r) <= 45
    r *= this.degree;
    // Possibly could call the gnu extension sincos
    s = Math.sin(r); c = Math.cos(r);
    switch (q & 3) {
      case 0:  sinx =  s; cosx =  c; break;
      case 1:  sinx =  c; cosx = -s; break;
      case 2:  sinx = -s; cosx = -c; break;
      default: sinx = -c; cosx =  s; break; // case 3
    }
    if (x !== 0) { sinx += 0; cosx += 0; }
    return {s: sinx, c: cosx};
  };

  /**
   * @summary Evaluate the atan2 function with the result in degrees
   * @param {number} y
   * @param {number} x
   * @returns atan2(y, x) in degrees, in the range (&minus;180&deg;
   *   180&deg;].
   */
  m.atan2d = function(y, x) {
    // In order to minimize round-off errors, this function rearranges the
    // arguments so that result of atan2 is in the range [-pi/4, pi/4] before
    // converting it to degrees and mapping the result to the correct
    // quadrant.
    var q = 0, t, ang;
    if (Math.abs(y) > Math.abs(x)) { t = x; x = y; y = t; q = 2; }
    if (x < 0) { x = -x; ++q; }
    // here x >= 0 and x >= abs(y), so angle is in [-pi/4, pi/4]
    ang = Math.atan2(y, x) / this.degree;
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
  };
})(GeographicLib.Math);

(function(
  /**
   * @exports GeographicLib/Accumulator
   * @description Accurate summation via the
   *   {@link module:GeographicLib/Accumulator.Accumulator Accumulator} class
   *   (mainly for internal use).
   */
  a, m) {

  /**
   * @class
   * @summary Accurate summation of many numbers.
   * @classdesc This allows many numbers to be added together with twice the
   *   normal precision.  In the documentation of the member functions, sum
   *   stands for the value currently held in the accumulator.
   * @param {number | Accumulator} [y = 0]  set sum = y.
   */
  a.Accumulator = function(y) {
    this.Set(y);
  };

  /**
   * @summary Set the accumulator to a number.
   * @param {number | Accumulator} [y = 0] set sum = y.
   */
  a.Accumulator.prototype.Set = function(y) {
    if (!y) y = 0;
    if (y.constructor === a.Accumulator) {
      this._s = y._s;
      this._t = y._t;
    } else {
      this._s = y;
      this._t = 0;
    }
  };

  /**
   * @summary Add a number to the accumulator.
   * @param {number} [y = 0] set sum += y.
   */
  a.Accumulator.prototype.Add = function(y) {
    // Here's Shewchuk's solution...
    // Accumulate starting at least significant end
    var u = m.sum(y, this._t),
        v = m.sum(u.s, this._s);
    u = u.t;
    this._s = v.s;
    this._t = v.t;
    // Start is _s, _t decreasing and non-adjacent.  Sum is now (s + t + u)
    // exactly with s, t, u non-adjacent and in decreasing order (except
    // for possible zeros).  The following code tries to normalize the
    // result.  Ideally, we want _s = round(s+t+u) and _u = round(s+t+u -
    // _s).  The follow does an approximate job (and maintains the
    // decreasing non-adjacent property).  Here are two "failures" using
    // 3-bit floats:
    //
    // Case 1: _s is not equal to round(s+t+u) -- off by 1 ulp
    // [12, -1] - 8 -> [4, 0, -1] -> [4, -1] = 3 should be [3, 0] = 3
    //
    // Case 2: _s+_t is not as close to s+t+u as it shold be
    // [64, 5] + 4 -> [64, 8, 1] -> [64,  8] = 72 (off by 1)
    //                    should be [80, -7] = 73 (exact)
    //
    // "Fixing" these problems is probably not worth the expense.  The
    // representation inevitably leads to small errors in the accumulated
    // values.  The additional errors illustrated here amount to 1 ulp of
    // the less significant word during each addition to the Accumulator
    // and an additional possible error of 1 ulp in the reported sum.
    //
    // Incidentally, the "ideal" representation described above is not
    // canonical, because _s = round(_s + _t) may not be true.  For
    // example, with 3-bit floats:
    //
    // [128, 16] + 1 -> [160, -16] -- 160 = round(145).
    // But [160, 0] - 16 -> [128, 16] -- 128 = round(144).
    //
    if (this._s === 0)          // This implies t == 0,
      this._s = u;              // so result is u
    else
      this._t += u;             // otherwise just accumulate u to t.
  };

  /**
   * @summary Return the result of adding a number to sum (but
   *   don't change sum).
   * @param {number} [y = 0] the number to be added to the sum.
   * @return sum + y.
   */
  a.Accumulator.prototype.Sum = function(y) {
    var b;
    if (!y)
      return this._s;
    else {
      b = new a.Accumulator(this);
      b.Add(y);
      return b._s;
    }
  };

  /**
   * @summary Set sum = &minus;sum.
   */
  a.Accumulator.prototype.Negate = function() {
    this._s *= -1;
    this._t *= -1;
  };
})(GeographicLib.Accumulator, GeographicLib.Math);
