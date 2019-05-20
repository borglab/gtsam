using System;
using NETGeographicLib;

namespace example_UTMUPS
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
                    string zonestr = UTMUPS.EncodeZone(zone, northp, true);
                    Console.WriteLine(String.Format("{0} {1} {2}", zonestr, x, y));
                }
                {
                    // Sample reverse calculation
                    string zonestr = "38N";
                    int zone;
                    bool northp;
                    UTMUPS.DecodeZone(zonestr, out zone, out northp);
                    double x = 444e3, y = 3688e3;
                    double lat, lon;
                    UTMUPS.Reverse(zone, northp, x, y, out lat, out lon, true);
                    Console.WriteLine(String.Format("{0} {1}", lat,lon));
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
