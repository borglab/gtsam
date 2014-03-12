using System;
using NETGeographicLib;

namespace example_MagneticModel
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                MagneticModel mag = new MagneticModel("wmm2010","");
                double lat = 27.99, lon = 86.93, h = 8820, t = 2012; // Mt Everest
                double Bx, By, Bz;
                mag.Field(t, lat,lon, h, out Bx, out By, out Bz);
                double H, F, D, I;
                MagneticModel.FieldComponents(Bx, By, Bz, out H, out F, out D, out I);
                Console.WriteLine(String.Format("{0} {1} {2} {3}", H, F, D, I));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
