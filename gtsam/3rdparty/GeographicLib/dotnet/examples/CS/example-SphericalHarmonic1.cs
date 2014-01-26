using System;
using NETGeographicLib;

namespace example_SphericalHarmonic1
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                int N = 3, N1 = 2;                  // The maximum degrees
                double[] ca = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
                double[] sa = {6, 5, 4, 3, 2, 1}; // sine coefficients
                double[] cb = {1, 2, 3, 4, 5, 6};
                double[] sb = {3, 2, 1};
                double a = 1;
                SphericalHarmonic1 h = new SphericalHarmonic1(ca, sa, N, cb, sb, N1, a, SphericalHarmonic1.Normalization.SCHMIDT);
                double tau = 0.1, x = 2, y = 3, z = 1;
                double v, vx, vy, vz;
                v = h.HarmonicSum(tau, x, y, z, out vx, out vy, out vz);
                Console.WriteLine(String.Format("{0} {1} {2} {3}", v, vx, vy, vz));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
