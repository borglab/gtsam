using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
  try {
    {
      // Sample forward calculation
      double lat = 57.64911, lon = 10.40744; // Jutland
      String^ georef;
      for (int prec = -1; prec <= 11; ++prec) {
        Georef::Forward(lat, lon, prec, georef);
        Console::WriteLine(String::Format("Precision: {0} Georef: {1}", prec, georef));
      }
    }
    {
      // Sample reverse calculation
      String^ georef = gcnew String("NKLN2444638946");
      double lat, lon;
      int prec;
      Georef::Reverse(georef->Substring(0, 2), lat, lon, prec, true);
      Console::WriteLine(String::Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon));
      Georef::Reverse(georef->Substring(0, 4), lat, lon, prec, true);
      Console::WriteLine(String::Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon));
      Georef::Reverse(georef, lat, lon, prec, true);
      Console::WriteLine(String::Format("Precision: {0} Latitude: {1} Longitude: {2}", prec, lat, lon));
    }
  }
  catch (GeographicErr^ e) {
    Console::WriteLine(String::Format("Caught Exception {0}", e->Message));
    return 1;
  }
  return 0;
}
