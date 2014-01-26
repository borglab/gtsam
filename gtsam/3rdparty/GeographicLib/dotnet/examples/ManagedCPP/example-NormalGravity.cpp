using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        NormalGravity^ grav = gcnew NormalGravity(NormalGravity::StandardModels::WGS84);
        double lat = 27.99, h = 8820; // Mt Everest
        double gammay, gammaz;
        grav->Gravity(lat, h, gammay, gammaz);
        Console::WriteLine(String::Format("{0} {1}", gammay, gammaz));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
