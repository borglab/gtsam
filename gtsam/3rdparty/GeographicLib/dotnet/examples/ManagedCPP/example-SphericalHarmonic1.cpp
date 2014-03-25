using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        int N = 3, N1 = 2;                  // The maximum degrees
        array<double>^ ca = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
        array<double>^ sa = {6, 5, 4, 3, 2, 1}; // sine coefficients
        array<double>^ cb = {1, 2, 3, 4, 5, 6};
        array<double>^ sb = {3, 2, 1};
        double a = 1;
        SphericalHarmonic1^ h = gcnew SphericalHarmonic1(ca, sa, N, cb, sb, N1, a, SphericalHarmonic1::Normalization::SCHMIDT);
        double tau = 0.1, x = 2, y = 3, z = 1;
        double v, vx, vy, vz;
        v = h->HarmonicSum(tau, x, y, z, vx, vy, vz);
        Console::WriteLine(String::Format("{0} {1} {2} {3}", v, vx, vy, vz));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine(String::Format("Caught exception: {0}", e->Message));
        return -1;
    }
    return 0;
}
