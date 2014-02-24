using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        MagneticModel^ mag = gcnew MagneticModel("wmm2010","");
        double lat = 27.99, lon = 86.93, h = 8820, t = 2012; // Mt Everest
        double Bx, By, Bz;
        mag->Field(t, lat,lon, h, Bx, By, Bz);
        double H, F, D, I;
        MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
        Console::WriteLine(String::Format("{0} {1} {2} {3}", H, F, D, I));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
