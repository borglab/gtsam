using namespace System;
using namespace NETGeographicLib;

int main(array<System::String ^> ^/*args*/)
{
    try {
        // Compare using Accumulator and ordinary summation for a sum of large and
        // small terms.
        double sum = 0;
        Accumulator^ acc = gcnew Accumulator();
        acc->Assign( 0.0 );
        sum += 1e20; sum += 1; sum += 2; sum += 100; sum += 5000; sum += -1e20;
        acc->Sum( 1e20 ); acc->Sum( 1 ); acc->Sum( 2 ); acc->Sum( 100 ); acc->Sum( 5000 ); acc->Sum( -1e20 );
        Console::WriteLine(String::Format("{0} {1}", sum, acc->Result()));
    }
    catch (GeographicErr^ e) {
        Console::WriteLine( String::Format(  "Caught exception: {0}", e->Message ) );
        return -1;
    }
    return 0;
}
