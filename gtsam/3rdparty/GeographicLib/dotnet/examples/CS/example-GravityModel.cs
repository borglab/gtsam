using System;
using NETGeographicLib;

namespace example_GravityModel
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                GravityModel grav = new GravityModel("egm96","");
                double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
                double gx, gy, gz;
                grav.Gravity(lat, lon, h, out gx, out gy, out gz);
                Console.WriteLine(String.Format("{0} {1} {2}", gx, gy, gz));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
