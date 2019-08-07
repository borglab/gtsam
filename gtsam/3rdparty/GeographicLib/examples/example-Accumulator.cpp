// Example of using the GeographicLib::Accumulator class

#include <iostream>
#include <exception>
#include <GeographicLib/Accumulator.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    // Compare using Accumulator and ordinary summation for a sum of large and
    // small terms.
    double sum = 0;
    Accumulator<> acc = 0;
    sum += 1e20; sum += 1; sum += 2; sum += 100; sum += 5000; sum += -1e20;
    acc += 1e20; acc += 1; acc += 2; acc += 100; acc += 5000; acc += -1e20;
    cout << sum << " " << acc() << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
