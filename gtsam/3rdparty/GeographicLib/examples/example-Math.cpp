// Example of using the GeographicLib::Math class

#include <iostream>
#include <exception>
#include <GeographicLib/Math.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    cout << Math::pi() << " " << Math::sq(Math::pi()) << "\n";
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
