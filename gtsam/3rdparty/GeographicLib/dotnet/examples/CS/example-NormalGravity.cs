using System;
using NETGeographicLib;

namespace example_NormalGravity
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                NormalGravity grav = new NormalGravity(NormalGravity.StandardModels.WGS84);
                double lat = 27.99, h = 8820; // Mt Everest
                double gammay, gammaz;
                grav.Gravity(lat, h, out gammay, out gammaz);
                Console.WriteLine(String.Format("{0} {1}", gammay, gammaz));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
