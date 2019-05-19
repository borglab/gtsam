using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using NETGeographicLib;

namespace example_RhumbLine
{
    class Program
    {
        static void Main(string[] args)
        {
          try {
            // Print waypoints between JFK and SIN
            Rhumb rhumb = new Rhumb(Constants.WGS84.MajorRadius, Constants.WGS84.Flattening, true);
            // Alternatively: const Rhumb& rhumb = Rhumb::WGS84();
            double
              lat1 = 40.640, lon1 = -73.779, // JFK
              lat2 =  1.359, lon2 = 103.989; // SIN
            double s12, azi12;
            rhumb.Inverse(lat1, lon1, lat2, lon2, out s12, out azi12);
            RhumbLine line = rhumb.Line(lat1, lon1, azi12);
            // Alternatively
            // const GeographicLib::RhumbLine line = rhumb.Line(lat1, lon1, azi1);
            double ds = 500e3;          // Nominal distance between points = 500 km
            int num = (int)Math.Ceiling(s12 / ds); // The number of intervals
            {
              // Use intervals of equal length
              ds = s12 / num;
              for (int i = 0; i <= num; ++i) {
                double lat, lon;
                line.Position(i * ds, out lat, out lon);
                        Console.WriteLine( "{0} {1} {2}", i, lat, lon );
              }
            }
          }
          catch (GeographicErr e) {
              Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
          }
        }
    }
}
