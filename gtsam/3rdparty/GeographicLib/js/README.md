# Geodesic routines from GeographicLib

This library is a JavaScript implementation of the geodesic routines
from [GeographicLib](https://geographiclib.sourceforge.io).  This solves the
direct and inverse geodesic problems for an ellipsoid of revolution.

Licensed under the MIT/X11 License; see
[LICENSE.txt](https://geographiclib.sourceforge.io/html/LICENSE.txt).

## Installation

```bash
$ npm install geographiclib
```

## Usage

In [node](https://nodejs.org), do
```javascript
var GeographicLib = require("geographiclib");
```

## Documentation

Full documentation is provided at
[https://geographiclib.sourceforge.io/1.49/js/](https://geographiclib.sourceforge.io/1.49/js/).

## Examples

```javascript
var GeographicLib = require("geographiclib"),
    geod = GeographicLib.Geodesic.WGS84, r;

// Find the distance from Wellington, NZ (41.32S, 174.81E) to
// Salamanca, Spain (40.96N, 5.50W)...
r = geod.Inverse(-41.32, 174.81, 40.96, -5.50);
console.log("The distance is " + r.s12.toFixed(3) + " m.");
// This prints "The distance is 19959679.267 m."

// Find the point 20000 km SW of Perth, Australia (32.06S, 115.74E)...
r = geod.Direct(-32.06, 115.74, 225, 20000e3);
console.log("The position is (" +
            r.lat2.toFixed(8) + ", " + r.lon2.toFixed(8) + ").");
// This prints "The position is (32.11195529, -63.95925278)."
```

## Authors

* algorithms + js code: Charles Karney (charles@karney.com)
* node.js port: Yurij Mikhalevich (0@39.yt)
