using System;
using NETGeographicLib;

namespace example_GravityCircle
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                GravityModel grav = new GravityModel("egm96", "");
                double lat = 27.99, lon0 = 86.93, h = 8820; // Mt Everest
                {
                    // Slow method of evaluating the values at several points on a circle of
                    // latitude.
                    for (int i = -5; i <= 5; ++i) {
                        double lon = lon0 + i * 0.2;
                        double gx, gy, gz;
                        grav.Gravity(lat, lon, h, out gx, out gy, out gz);
                        Console.WriteLine(String.Format("{0} {1} {2} {3}", lon, gx, gy, gz));
                    }
                }
                {
                    // Fast method of evaluating the values at several points on a circle of
                    // latitude using GravityCircle.
                    GravityCircle circ = grav.Circle(lat, h, GravityModel.Mask.ALL);
                    for (int i = -5; i <= 5; ++i) {
                        double lon = lon0 + i * 0.2;
                        double gx, gy, gz;
                        circ.Gravity(lon, out gx, out gy, out gz);
                        Console.WriteLine(String.Format("{0} {1} {2} {3}", lon, gx, gy, gz));
                    }
                }
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
