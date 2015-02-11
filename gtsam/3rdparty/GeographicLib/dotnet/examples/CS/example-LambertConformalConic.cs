using System;
using NETGeographicLib;

namespace example_LambertConformalConic
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                // Define the Pennsylvania South state coordinate system EPSG:3364
                // http://www.spatialreference.org/ref/epsg/3364/
                const double
                    lat1 = 40 + 58/60.0, lat2 = 39 + 56/60.0, // standard parallels
                    k1 = 1,                                   // scale
                    lat0 = 39 + 20/60.0, lon0 =-77 - 45/60.0, // origin
                    fe = 600000, fn = 0;                      // false easting and northing
                // Set up basic projection
                LambertConformalConic PASouth = new LambertConformalConic( Constants.WGS84.MajorRadius,
                                                                           Constants.WGS84.Flattening,
                                                                           lat1, lat2, k1);
                double x0, y0;
                // Transform origin point
                PASouth.Forward(lon0, lat0, lon0, out x0, out y0);
                x0 -= fe; y0 -= fn;
                {
                    // Sample conversion from geodetic to PASouth grid
                    double lat = 39.95, lon = -75.17;    // Philadelphia
                    double x, y;
                    PASouth.Forward(lon0, lat, lon, out x, out y);
                    x -= x0; y -= y0;
                    Console.WriteLine( String.Format("{0} {1}", x, y) );
                }
                {
                    // Sample conversion from PASouth grid to geodetic
                    double x = 820e3, y = 72e3;
                    double lat, lon;
                    x += x0; y += y0;
                    PASouth.Reverse(lon0, x, y, out lat, out lon);
                    Console.WriteLine( String.Format("{0} {1}", lat, lon) );
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine( String.Format("Caught exception: {0}", e.Message) );
            }
        }
    }
}
