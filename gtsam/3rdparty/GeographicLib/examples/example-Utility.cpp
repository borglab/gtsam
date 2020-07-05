// Example of using the GeographicLib::Utility class

#include <iostream>
#include <exception>
#include <GeographicLib/Utility.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    int
      d1 = Utility::day(1939, 9, 3),  // Britain declares war on Germany
      d2 = Utility::day(1945, 8, 15); // Japan surrenders
    cout << d2 - d1 << "\n";          // Length of Second World War for Britain
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
