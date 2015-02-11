using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        Geocentric^ earth = gcnew Geocentric( Constants::WGS84::MajorRadius,
                                              Constants::WGS84::Flattening);
        // Alternatively: Geocentric earth = new Geocentric();
        {
            // Sample forward calculation
            double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
            double X, Y, Z;
            earth->Forward(lat, lon, h, X, Y, Z);
            Console::WriteLine( String::Format( "{0} {1} {2}",
                Math::Floor(X / 1000 + 0.5),
                Math::Floor(Y / 1000 + 0.5),
                Math::Floor(Z / 1000 + 0.5) ) );
        }
        {
            // Sample reverse calculation
            double X = 302e3, Y = 5636e3, Z = 2980e3;
            double lat, lon, h;
            earth->Reverse(X, Y, Z, lat, lon, h);
            Console::WriteLine(String::Format("{0} {1} {2}", lat, lon, h));
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine( String::Format( "Caught exception: {0}", e->Message ) );
        return -1;
    }
    return 0;
}
