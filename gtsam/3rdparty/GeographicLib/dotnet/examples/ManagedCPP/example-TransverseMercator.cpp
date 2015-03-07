using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        TransverseMercator^ proj = gcnew TransverseMercator(); // WGS84
        double lon0 = -75;          // Central meridian for UTM zone 18
        {
            // Sample forward calculation
            double lat = 40.3, lon = -74.7; // Princeton, NJ
            double x, y;
            proj->Forward(lon0, lat, lon, x, y);
            Console::WriteLine(String::Format("{0} {1}", x, y));
        }
        {
            // Sample reverse calculation
            double x = 25e3, y = 4461e3;
            double lat, lon;
            proj->Reverse(lon0, x, y, lat, lon);
            Console::WriteLine(String::Format("{0} {1}", lat, lon));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
