using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        // See also example-GeoCoords.cpp
        {
            // Sample forward calculation
            double lat = 33.3, lon = 44.4; // Baghdad
            int zone;
            bool northp;
            double x, y;
            UTMUPS::Forward(lat, lon, zone, northp, x, y, -1, true);
            String^ mgrs;
            MGRS::Forward(zone, northp, x, y, lat, 5, mgrs);
            Console::WriteLine(mgrs);
        }
        {
            // Sample reverse calculation
            String^ mgrs = "38SMB4488";
            int zone, prec;
            bool northp;
            double x, y;
            MGRS::Reverse(mgrs, zone, northp, x, y, prec, true);
            double lat, lon;
            UTMUPS::Reverse(zone, northp, x, y, lat, lon, true);
            Console::WriteLine(String::Format("Latitude: {0} Longitude: {1}", lat, lon));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
