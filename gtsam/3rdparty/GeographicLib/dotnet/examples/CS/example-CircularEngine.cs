using System;
using NETGeographicLib;

namespace example_CircularEngine
{
    class Program
    {
        static void Main(string[] args)
        {
            // This computes the same value as example-SphericalHarmonic.cpp using a
            // CircularEngine (which will be faster if many values on a circle of
            // latitude are to be found).
            try {
                int N = 3;                  // The maxium degree
                double[] ca = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
                double[] sa = {6, 5, 4, 3, 2, 1}; // sine coefficients
                double a = 1;
                SphericalHarmonic h = new SphericalHarmonic(ca, sa, N, a, SphericalHarmonic.Normalization.SCHMIDT);
                double x = 2, y = 3, z = 1, p = Math.Sqrt(x*x+y*y);
                CircularEngine circ = h.Circle(p, z, true);
                double v, vx, vy, vz;
                v = circ.LongitudeSum(x/p, y/p, out vx, out vy, out vz);
                Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
