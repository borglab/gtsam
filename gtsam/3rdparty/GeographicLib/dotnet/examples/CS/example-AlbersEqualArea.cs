using System;
using NETGeographicLib;

namespace example_AlbersEqualArea
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                const double
                    lat1 = 40 + 58/60.0, lat2 = 39 + 56/60.0, // standard parallels
                    k1 = 1,                                   // scale
                    lon0 = -77 - 45/60.0;                     // Central meridian
                // Set up basic projection
                AlbersEqualArea albers = new AlbersEqualArea( Constants.WGS84.MajorRadius,
                                                              Constants.WGS84.Flattening,
                                                              lat1, lat2, k1);
                {
                    // Sample conversion from geodetic to Albers Equal Area
                    double lat = 39.95, lon = -75.17;    // Philadelphia
                    double x, y;
                    albers.Forward(lon0, lat, lon, out x, out y);
                    Console.WriteLine( String.Format("X: {0} Y: {1}", x, y ) );
                }
                {
                    // Sample conversion from Albers Equal Area grid to geodetic
                    double x = 220e3, y = -53e3;
                    double lat, lon;
                    albers.Reverse(lon0, x, y, out lat, out lon);
                    Console.WriteLine( String.Format("Latitude: {0} Longitude: {1}", lat, lon ) );
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine( String.Format( "Caught exception: {0}", e.Message ) );
            }
        }
    }
}
