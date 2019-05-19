using System;
using NETGeographicLib;

namespace example_Georef
{
  class Program
  {
    static void Main(string[] args)
    {
      try {
        {
          // Sample forward calculation
          double lat = 57.64911, lon = 10.40744; // Jutland
          string georef;
          for (int prec = -1; prec <= 11; ++prec) {
            Georef.Forward(lat, lon, prec, out georef);
            Console.WriteLine(String.Format("Precision: {0} Georef: {1}", prec, georef));
          }
        }
        {
          // Sample reverse calculation
          string georef = "NKLN2444638946";
          double lat, lon;
          int prec;
          Georef.Reverse(georef.Substring(0, 2), out lat, out lon, out prec, true);
          Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon));
          Georef.Reverse(georef.Substring(0, 4), out lat, out lon, out prec, true);
          Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon));
          Georef.Reverse(georef, out lat, out lon, out prec, true);
          Console.WriteLine(String.Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon));
        }
      }
      catch (GeographicErr e) {
        Console.WriteLine(String.Format("Caught Exception {0}", e.Message));
      }
    }
  }
}
