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
            String^ zonestr = UTMUPS::EncodeZone(zone, northp, true);
            Console::WriteLine(String::Format("{0} {1} {2}", zonestr, x, y));
        }
        {
            // Sample reverse calculation
            String^ zonestr = "38N";
            int zone;
            bool northp;
            UTMUPS::DecodeZone(zonestr, zone, northp);
            double x = 444e3, y = 3688e3;
            double lat, lon;
            UTMUPS::Reverse(zone, northp, x, y, lat, lon, true);
            Console::WriteLine(String::Format("{0} {1}", lat,lon));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
