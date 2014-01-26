using namespace System;
using namespace NETGeographicLib;

int main() {
  Geodesic^ geod = gcnew Geodesic();
  // Distance from JFK to LHR
  double
    lat1 = 40.6, lon1 = -73.8, // JFK Airport
    lat2 = 51.6, lon2 = -0.5;  // LHR Airport
  double s12;
  geod->Inverse(lat1, lon1, lat2, lon2, s12);
  Console::WriteLine( s12 / 1000 + " km" );
  return 0;
}
