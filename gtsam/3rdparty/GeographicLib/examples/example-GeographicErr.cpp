// Example of using the GeographicLib::GeographicErr class

#include <iostream>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    throw GeographicErr("Test throwing an exception");
  }
  catch (const GeographicErr& e) {
    cout << "Caught exception: " << e.what() << "\n";
  }
}
