using System;
using NETGeographicLib;

namespace example_Ellipsoid
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                Ellipsoid wgs84 = new Ellipsoid( Constants.WGS84.MajorRadius,
                                                 Constants.WGS84.Flattening);
                // Alternatively: Ellipsoid wgs84 = new Ellipsoid();
                Console.WriteLine( String.Format(
                    "The latitude half way between the equator and the pole is {0}",
                     wgs84.InverseRectifyingLatitude(45)) );
                Console.WriteLine( String.Format(
                    "Half the area of the ellipsoid lies between latitudes +/- {0}",
                    wgs84.InverseAuthalicLatitude(30))); ;
                Console.WriteLine( String.Format(
                    "The northernmost edge of a square Mercator map is at latitude {0}",
                    wgs84.InverseIsometricLatitude(180)));
            }
            catch (GeographicErr e) {
                Console.WriteLine( String.Format( "Caught exception: {0}", e.Message ) );
            }
        }
    }
}
