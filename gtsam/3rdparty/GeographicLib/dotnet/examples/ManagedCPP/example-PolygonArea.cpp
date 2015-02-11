using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        Geodesic^ geod = gcnew Geodesic();  // WGS84
        PolygonArea^ poly = gcnew PolygonArea(geod, true);
        poly->AddPoint( 52,  0);     // London
        poly->AddPoint( 41,-74);     // New York
        poly->AddPoint(-23,-43);     // Rio de Janeiro
        poly->AddPoint(-26, 28);     // Johannesburg
        double perimeter, area;
        unsigned int n = poly->Compute(false, true, perimeter, area);
        Console::WriteLine(String::Format("{0} {1} {2}", n, perimeter, area));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
