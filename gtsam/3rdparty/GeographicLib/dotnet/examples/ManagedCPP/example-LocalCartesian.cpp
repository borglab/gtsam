using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        Geocentric^ earth = gcnew Geocentric();
        const double lat0 = 48 + 50/60.0, lon0 = 2 + 20/60.0; // Paris
        LocalCartesian^ proj = gcnew LocalCartesian(lat0, lon0, 0, earth);
        {
            // Sample forward calculation
            double lat = 50.9, lon = 1.8, h = 0; // Calais
            double x, y, z;
            proj->Forward(lat, lon, h, x, y, z);
            Console::WriteLine(String::Format("{0} {1} {2}", x, y, z));
        }
        {
            // Sample reverse calculation
            double x = -38e3, y = 230e3, z = -4e3;
            double lat, lon, h;
            proj->Reverse(x, y, z, lat, lon, h);
            Console::WriteLine(String::Format("{0} {1} {2}", lat, lon, h));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
