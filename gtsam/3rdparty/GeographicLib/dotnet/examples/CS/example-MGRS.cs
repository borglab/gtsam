using System;
using NETGeographicLib;

namespace example_MGRS
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                // See also example-GeoCoords.cpp
                {
                    // Sample forward calculation
                    double lat = 33.3, lon = 44.4; // Baghdad
                    int zone;
                    bool northp;
                    double x, y;
                    UTMUPS.Forward(lat, lon, out zone, out northp, out x, out y, -1, true);
                    string mgrs;
                    MGRS.Forward(zone, northp, x, y, lat, 5, out mgrs);
                    Console.WriteLine(mgrs);
                }
                {
                    // Sample reverse calculation
                    string mgrs = "38SMB4488";
                    int zone, prec;
                    bool northp;
                    double x, y;
                    MGRS.Reverse(mgrs, out zone, out northp, out x, out y, out prec, true);
                    double lat, lon;
                    UTMUPS.Reverse(zone, northp, x, y, out lat, out lon, true);
                    Console.WriteLine(String.Format("Latitude: {0} Longitude: {1}", lat, lon));
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
