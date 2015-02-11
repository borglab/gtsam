using System;
using NETGeographicLib;

namespace example_LocalCartesian
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                Geocentric earth = new Geocentric();
                const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
                LocalCartesian proj = new LocalCartesian(lat0, lon0, 0, earth);
                {
                    // Sample forward calculation
                    double lat = 50.9, lon = 1.8, h = 0; // Calais
                    double x, y, z;
                    proj.Forward(lat, lon, h, out x, out y, out z);
                    Console.WriteLine(String.Format("{0} {1} {2}", x, y, z));
                }
                {
                    // Sample reverse calculation
                    double x = -38e3, y = 230e3, z = -4e3;
                    double lat, lon, h;
                    proj.Reverse(x, y, z, out lat, out lon, out h);
                    Console.WriteLine(String.Format("{0} {1} {2}", lat, lon, h));
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
