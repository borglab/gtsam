using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        GravityModel^ grav = gcnew GravityModel("egm96","");
        double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
        double gx, gy, gz;
        grav->Gravity(lat, lon, h, gx, gy, gz);
        Console::WriteLine(String::Format("{0} {1} {2}", gx, gy, gz));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
