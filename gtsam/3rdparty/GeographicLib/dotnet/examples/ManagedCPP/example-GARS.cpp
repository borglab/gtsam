using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
  try {
    {
      // Sample forward calculation
      double lat = 57.64911, lon = 10.40744; // Jutland
      String^ gars;
      for (int prec = 0; prec <= 2; ++prec) {
        GARS::Forward(lat, lon, prec, gars);
        Console::WriteLine(String::Format("Precision: {0} GARS: {1}", prec, gars));
      }
    }
    {
      // Sample reverse calculation
      String^ gars = gcnew String("381NH45");
      double lat, lon;
      for (int len = 5; len <= gars->Length; ++len) {
        int prec;
        GARS::Reverse(gars->Substring(0, len), lat, lon, prec, true);
        Console::WriteLine(String::Format("Precision: {0} Latitude: {1} Longitude {2}", prec, lat, lon));
      }
    }
  }
  catch (GeographicErr^ e) {
    Console::WriteLine(String::Format("Caught Exception {0}", e->Message));
    return 1;
  }
  return 0;
}
