using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        {
            // Sample forward calculation from
            // A guide to coordinate systems in Great Britain
            double
            lat = DMS::Decode(52,39,27.2531),
            lon = DMS::Decode( 1,43, 4.5177);
            double x, y;
            OSGB::Forward(lat, lon, x, y);
            String^ gridref;
            OSGB::GridReference(x, y, 2, gridref);
            Console::WriteLine(String::Format("{0} {1} {2}", x, y, gridref));
        }
        {
            // Sample reverse calculation
            String^ gridref = "TG5113";
            double x, y;
            int prec;
            OSGB::GridReference(gridref, x, y, prec, true);
            double lat, lon;
            OSGB::Reverse(x, y, lat, lon);
            Console::WriteLine(String::Format("{0} {1} {2}", prec, lat, lon));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return 0;
    }
    return 0;
}
