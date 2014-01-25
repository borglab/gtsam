using System;
using NETGeographicLib;

namespace example_GeoCoords
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                // Miscellaneous conversions
                double lat = 33.3, lon = 44.4;
                GeoCoords c = new GeoCoords(lat, lon, -1);
                Console.WriteLine(c.MGRSRepresentation(-3));
                c.Reset("18TWN0050", true, false);
                Console.WriteLine(c.DMSRepresentation(0, false, 0));
                Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", c.Latitude, c.Longitude));
                c.Reset("1d38'W 55d30'N", true, false);
                Console.WriteLine(c.GeoRepresentation(0, false));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
