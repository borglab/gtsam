using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        Ellipsoid^ wgs84 = gcnew Ellipsoid( Constants::WGS84::MajorRadius,
                                            Constants::WGS84::Flattening );
        // Alternatively: Ellipsoid^ wgs84 = gcnew Ellipsoid();
        Console::WriteLine( String::Format(
            "The latitude half way between the equator and the pole is {0}",
                wgs84->InverseRectifyingLatitude(45)) );
        Console::WriteLine( String::Format(
            "Half the area of the ellipsoid lies between latitudes +/- {0}",
            wgs84->InverseAuthalicLatitude(30))); ;
        Console::WriteLine( String::Format(
            "The northernmost edge of a square Mercator map is at latitude {0}",
            wgs84->InverseIsometricLatitude(180)));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine( String::Format( "Caught exception: {0}", e->Message ) );
        return -1;
    }
    return 0;
}
