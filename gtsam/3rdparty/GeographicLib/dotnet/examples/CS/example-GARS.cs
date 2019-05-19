using System;
using NETGeographicLib;

namespace example_GARS
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
              {
                // Sample forward calculation
                double lat = 57.64911, lon = 10.40744; // Jutland
                string gars;
                for (int prec = 0; prec <= 2; ++prec) {
                  GARS.Forward(lat, lon, prec, out gars);
                  Console.WriteLine(String.Format("Precision: {0} GARS: {1}", prec, gars));
                }
              }
              {
                // Sample reverse calculation
                string gars = "381NH45";
                double lat, lon;
                for (int len = 5; len <= gars.Length; ++len) {
                  int prec;
                  GARS.Reverse(gars.Substring(0, len), out lat, out lon, out prec, true);
                  Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude {2}", prec, lat, lon));
                }
              }
            }
            catch (GeographicErr e) {
              Console.WriteLine(String.Format("Caught Exception {0}", e.Message));
            }
        }
    }
}
