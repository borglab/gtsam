using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        int N = 3;                  // The maximum degree
        array<double>^ ca = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
        array<double>^ sa = {6, 5, 4, 3, 2, 1}; // sine coefficients
        double a = 1;
        SphericalHarmonic^ h = gcnew SphericalHarmonic(ca, sa, N, a, SphericalHarmonic::Normalization::SCHMIDT);
        double x = 2, y = 3, z = 1;
        double v, vx, vy, vz;
        v = h->HarmonicSum(x, y, z, vx, vy, vz);
        Console::WriteLine(String::Format("{0} {1} {2} {3}", v, vx, vy, vz));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
