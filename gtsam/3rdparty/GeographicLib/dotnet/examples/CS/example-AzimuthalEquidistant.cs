using System;
using NETGeographicLib;

namespace example_AzimuthalEquidistant
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                Geodesic geod = new Geodesic(); // WGS84
                const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
                AzimuthalEquidistant proj = new AzimuthalEquidistant(geod);
                {
                    // Sample forward calculation
                    double lat = 50.9, lon = 1.8; // Calais
                    double x, y;
                    proj.Forward(lat0, lon0, lat, lon, out x, out y);
                    Console.WriteLine( String.Format("X: {0} Y: {1}", x, y ) );
                }
                {
                    // Sample reverse calculation
                    double x = -38e3, y = 230e3;
                    double lat, lon;
                    proj.Reverse(lat0, lon0, x, y, out lat, out lon);
                    Console.WriteLine( String.Format("Latitude: {0} Longitude: {1}", lat, lon ) );
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine( String.Format( "Caught exception: {0}", e.Message ) );
            }
        }
    }
}
