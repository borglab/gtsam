using System;
using NETGeographicLib;

namespace example_PolygonArea
{
    class Program
    {
        static void Main(string[] args)
        {
            try {
                Geodesic geod = new Geodesic();  // WGS84
                PolygonArea poly = new PolygonArea(geod, true);
                poly.AddPoint( 52,  0);     // London
                poly.AddPoint( 41,-74);     // New York
                poly.AddPoint(-23,-43);     // Rio de Janeiro
                poly.AddPoint(-26, 28);     // Johannesburg
                double perimeter, area;
                uint n = poly.Compute(false, true, out perimeter, out area);
                Console.WriteLine(String.Format("{0} {1} {2}", n, perimeter, area));
            }
            catch (GeographicErr e) {
                Console.WriteLine(String.Format("Caught exception: {0}", e.Message));
            }
        }
    }
}
