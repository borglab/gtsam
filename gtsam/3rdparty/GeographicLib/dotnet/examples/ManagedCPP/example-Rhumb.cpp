using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/
)
{
  try {
    Rhumb^ rhumb = gcnew Rhumb(Constants::WGS84::MajorRadius, Constants::WGS84::Flattening, true);
    // Alternatively: const Rhumb& rhumb = Rhumb::WGS84();
    {
      // Sample direct calculation, travelling about NE from JFK
      double lat1 = 40.6, lon1 = -73.8, s12 = 5.5e6, azi12 = 51;
      double lat2, lon2;
      rhumb->Direct(lat1, lon1, azi12, s12, lat2, lon2);
      Console::WriteLine( "{0} {1}", lat2, lon2 );
    }
    {
      // Sample inverse calculation, JFK to LHR
      double
        lat1 = 40.6, lon1 = -73.8, // JFK Airport
        lat2 = 51.6, lon2 = -0.5;  // LHR Airport
      double s12, azi12;
      rhumb->Inverse(lat1, lon1, lat2, lon2, s12, azi12);
      Console::WriteLine( "{0} {1}", s12, azi12 );
    }
  }
  catch (GeographicErr^ e) {
      Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
      return -1;
  }
  return 0;
}
