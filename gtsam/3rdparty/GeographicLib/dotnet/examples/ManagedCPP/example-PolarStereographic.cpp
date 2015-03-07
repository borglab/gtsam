using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        PolarStereographic^ proj = gcnew PolarStereographic(); // WGS84
        bool northp = true;
        {
            // Sample forward calculation
            double lat = 61.2, lon = -149.9; // Anchorage
            double x, y;
            proj->Forward(northp, lat, lon, x, y);
            Console::WriteLine(String::Format("{0} {1}", x, y));
        }
        {
            // Sample reverse calculation
            double x = -1637e3, y = 2824e3;
            double lat, lon;
            proj->Reverse(northp, x, y, lat, lon);
            Console::WriteLine(String::Format("{0} {1}", lat, lon));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
