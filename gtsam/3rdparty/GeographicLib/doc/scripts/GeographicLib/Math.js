/**
 * Math.js
 * Transcription of Math.hpp, Constants.hpp, and Accumulator.hpp into
 * JavaScript.
 *
 * Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

var GeographicLib; if (!GeographicLib) GeographicLib = {};

GeographicLib.Math = {};

GeographicLib.Math.sq = function(x) { return x * x; }

GeographicLib.Math.hypot = function(x, y) {
  x = Math.abs(x);
  y = Math.abs(y);
  var a = Math.max(x, y), b = Math.min(x, y) / (a ? a : 1);
  return a * Math.sqrt(1 + b * b);
}

GeographicLib.Math.cbrt = function(x) {
  var y = Math.pow(Math.abs(x), 1/3);
  return x < 0 ? -y : y;
}

GeographicLib.Math.log1p = function(x) {
  var
  y = 1 + x,
  z = y - 1;
  // Here's the explanation for this magic: y = 1 + z, exactly, and z
  // approx x, thus log(y)/z (which is nearly constant near z = 0) returns
  // a good approximation to the true log(1 + x)/x.  The multiplication x *
  // (log(y)/z) introduces little additional error.
  return z == 0 ? x : x * Math.log(y) / z;
}

GeographicLib.Math.atanh = function(x) {
  var y = Math.abs(x);          // Enforce odd parity
  y = GeographicLib.Math.log1p(2 * y/(1 - y))/2;
  return x < 0 ? -y : y;
}

GeographicLib.Math.sum = function(u, v) {
  var
  s = u + v,
  up = s - v,
  vpp = s - up;
  up -= u;
  vpp -= v;
  t = -(up + vpp);
  // u + v =       s      + t
  //       = round(u + v) + t
  return {s: s, t: t};
}

GeographicLib.Math.AngNormalize = function(x) {
  // Place angle in [-180, 180).  Assumes x is in [-540, 540).
  return x >= 180 ? x - 360 : (x < -180 ? x + 360 : x);
}

GeographicLib.Math.AngNormalize2 = function(x) {
  // Place arbitrary angle in [-180, 180).
  return GeographicLib.Math.AngNormalize(x % 360);
}

GeographicLib.Math.AngDiff = function(x, y) {
  // Compute y - x and reduce to [-180,180] accurately.
  // This is the same logic as the Accumulator class uses.
  var
  d = y - x,
  yp = d + x,
  xpp = yp - d;
  yp -= y;
  xpp -= x;
  var t =  xpp - yp;
  // y - x =       d      + t
  //       = round(y - x) + t
  if ((d - 180) + t > 0)        // y - x > 180
    d -= 360;                   // exact
  else if ((d + 180) + t <= 0)  // y - x <= -180
    d += 360;                   // exact
  return d + t;
}

GeographicLib.Math.epsilon = Math.pow(0.5, 52);
GeographicLib.Math.degree = Math.PI/180;
GeographicLib.Math.digits = 53;

GeographicLib.Constants = {};
GeographicLib.Constants.WGS84 = { a: 6378137, f: 1/298.257223563 };

GeographicLib.Accumulator = {};
(function() {
  a = GeographicLib.Accumulator;
  var m = GeographicLib.Math;

  a.Accumulator = function(y) {
    this.Set(y);
  }

  a.Accumulator.prototype.Set = function(y) {
    if (!y) y = 0;
    if (y.constructor == a.Accumulator) {
      this._s = y._s;
      this._t = y._t;
    } else {
      this._s = y;
      this._t = 0;
    }
  }

  a.Accumulator.prototype.Add = function(y) {
    // Here's Shewchuk's solution...
    // Accumulate starting at least significant end
    var u = m.sum(y, this._t);
    var v = m.sum(u.s, this._s);
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
    if (this._s == 0)           // This implies t == 0,
      this._s = u;              // so result is u
    else
      this._t += u;             // otherwise just accumulate u to t.
  }

  a.Accumulator.prototype.Sum = function(y) {
    if (!y)
      return this._s;
    else {
      var b = new a.Accumulator(this);
      b.Add(y);
      return b._s;
    }
  }

  a.Accumulator.prototype.Negate = function() {
    this._s *= -1;
    this._t *= -1;
  }

})();
