using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        // Print waypoints between JFK and SIN
        Geodesic^ geod = gcnew Geodesic(); // WGS84
        double
            lat1 = 40.640, lon1 = -73.779, // JFK
            lat2 =  1.359, lon2 = 103.989; // SIN
        double s12, azi1, azi2,
            a12 = geod->Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2);
        GeodesicLine^ line = gcnew GeodesicLine(geod, lat1, lon1, azi1, Mask::ALL);
        // Alternatively
        // const GeographicLib::GeodesicLine line = geod.Line(lat1, lon1, azi1);
        double ds0 = 500e3;     // Nominal distance between points = 500 km
        int num = int(Math::Ceiling(s12 / ds0)); // The number of intervals
        {
            // Use intervals of equal length
            double ds = s12 / num;
            for (int i = 0; i <= num; ++i) {
                double lat, lon;
                line->Position(i * ds, lat, lon);
                Console::WriteLine( String::Format( "i: {0} Latitude: {1} Longitude: {2}", i, lat, lon ));
            }
        }
        {
            // Slightly faster, use intervals of equal arc length
            double da = a12 / num;
            for (int i = 0; i <= num; ++i) {
                double lat, lon;
                line->ArcPosition(i * da, lat, lon);
                Console::WriteLine( String::Format( "i: {0} Latitude: {1} Longitude: {2}", i, lat, lon ));
            }
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
