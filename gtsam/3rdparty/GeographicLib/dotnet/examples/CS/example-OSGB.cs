using System;
using NETGeographicLib;

namespace example_OSGB
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                {
                    // Sample forward calculation from
                    // A guide to coordinate systems in Great Britain
                    double
                    lat = DMS.Decode(52,39,27.2531),
                    lon = DMS.Decode( 1,43, 4.5177);
                    double x, y;
                    OSGB.Forward(lat, lon, out x, out y);
                    string gridref;
                    OSGB.GridReference(x, y, 2, out gridref);
                    Console.WriteLine(String.Format("{0} {1} {2}", x, y, gridref));
                }
                {
                    // Sample reverse calculation
                    string gridref = "TG5113";
                    double x, y;
                    int prec;
                    OSGB.GridReference(gridref, out x, out y, out prec, true);
                    double lat, lon;
                    OSGB.Reverse(x, y, out lat, out lon);
                    Console.WriteLine(String.Format("{0} {1} {2}", prec, lat, lon));
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
