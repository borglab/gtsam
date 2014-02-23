using System;
using NETGeographicLib;

namespace example_SphericalHarmonic
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                int N = 3;                  // The maximum degree
                double[] ca = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
                double[] sa = {6, 5, 4, 3, 2, 1}; // sine coefficients
                double a = 1;
                SphericalHarmonic h = new SphericalHarmonic(ca, sa, N, a, SphericalHarmonic.Normalization.SCHMIDT);
                double x = 2, y = 3, z = 1;
                double v, vx, vy, vz;
                v = h.HarmonicSum(x, y, z, out vx, out vy, out vz);
                Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
