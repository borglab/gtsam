using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        {
            // Sample forward calculation
            double lat = 57.64911, lon = 10.40744; // Jutland (the wikipedia example)
            String^ geohash;
            int maxlen = Geohash::GeohashLength(1.0e-5);
            for (int len = 0; len <= maxlen; ++len) {
                Geohash::Forward(lat, lon, len, geohash);
                Console::WriteLine( geohash );
            }
        }
        {
            // Sample reverse calculation
            String^ geohash = "u4pruydqqvj";
            double lat, lon;
            for (int i = 0; i <= geohash->Length; ++i) {
                int len;
                Geohash::Reverse(geohash->Substring(0, i), lat, lon, len, true);
                Console::WriteLine(String::Format("Length: {0} Latitude: {1} Longitude: {2}", len, lat, lon ) );
            }
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
