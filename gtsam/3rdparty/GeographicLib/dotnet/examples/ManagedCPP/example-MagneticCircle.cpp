using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        MagneticModel^ mag = gcnew MagneticModel("wmm2010","");
        double lat = 27.99, lon0 = 86.93, h = 8820, t = 2012; // Mt Everest
        {
            // Slow method of evaluating the values at several points on a circle of
            // latitude.
            for (int i = -5; i <= 5; ++i) {
                double lon = lon0 + i * 0.2;
                double Bx, By, Bz;
                mag->Field(t, lat, lon, h, Bx, By, Bz);
                Console::WriteLine(String::Format("{0} {1} {2} {3}", lon, Bx, By, Bz));
            }
        }
        {
            // Fast method of evaluating the values at several points on a circle of
            // latitude using MagneticCircle.
            MagneticCircle^ circ = mag->Circle(t, lat, h);
            for (int i = -5; i <= 5; ++i) {
                double lon = lon0 + i * 0.2;
                double Bx, By, Bz;
                circ->Field(lon, Bx, By, Bz);
                Console::WriteLine(String::Format("{0} {1} {2} {3}", lon, Bx, By, Bz));
            }
        }
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
