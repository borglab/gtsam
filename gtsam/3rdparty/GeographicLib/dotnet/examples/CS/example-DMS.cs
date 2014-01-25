using System;
using NETGeographicLib;

namespace example_DMS
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
            {
                string dms = "30d14'45.6\"S";
                DMS.Flag type;
                double ang = DMS.Decode(dms, out type);
                Console.WriteLine(String.Format("Type: {0} String: {1}", type, ang));
            }
            {
                double ang = -30.245715;
                string dms = DMS.Encode(ang, 6, DMS.Flag.LATITUDE, 0);
                Console.WriteLine(String.Format("Latitude: {0}", dms));
            }
            }
            catch (GeographicErr e) {
                Console.WriteLine( String.Format( "Caught exception: {0}", e.Message ) );
            }
        }
    }
}
