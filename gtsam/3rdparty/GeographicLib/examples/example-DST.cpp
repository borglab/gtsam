// Example of using the GeographicLib::DST class

#include <iostream>
#include <exception>
#include <vector>
#include <GeographicLib/Math.hpp>
#include <GeographicLib/DST.hpp>

using namespace std;
using namespace GeographicLib;

class sawtooth {
private:
  double _a;
public:
  sawtooth(double a) : _a(a) {}
  // only called for x in (0, pi/2].  DST assumes function is periodic, period
  // 2*pi, is odd about 0, and is even about pi/2.
  double operator()(double x) const { return _a * x; }
};

int main() {
  try {
    sawtooth f(Math::pi()/4);
    DST dst;
    int N = 5, K = 2*N;
    vector<double> tx(N), txa(2*N);
    dst.reset(N);
    dst.transform(f, tx.data());
    cout << "Transform of sawtooth based on " << N << " points\n"
         << "approx 1, -1/9, 1/25, -1/49, ...\n";
    for (int i = 0; i < min(K,N); ++i) {
      int j = (2*i+1)*(2*i+1)*(1-((i&1)<<1));
      cout << i << " " << tx[i] << " " << tx[i]*j << "\n";
    }
    tx.resize(2*N);
    dst.refine(f, tx.data());
    cout << "Add another " << N << " points\n";
    for (int i = 0; i < min(K,2*N); ++i) {
      int j = (2*i+1)*(2*i+1)*(1-((i&1)<<1));
      cout << i << " " << tx[i] << " " << tx[i]*j << "\n";
    }
    dst.reset(2*N);
    dst.transform(f, txa.data());
    cout << "Retransform of sawtooth based on " << 2*N << " points\n";
    for (int i = 0; i < min(K,2*N); ++i) {
      int j = (2*i+1)*(2*i+1)*(1-((i&1)<<1));
      cout << i << " " << txa[i] << " " << txa[i]*j << "\n";
    }
    cout << "Table of values and integral\n";
    for (int i = 0; i <= K; ++i) {
      double x = i*Math::pi()/(2*K), sinx = sin(x), cosx = cos(x);
      cout << x << " " << f(x) << " "
           << DST::eval(sinx, cosx, txa.data(), 2*N) << " "
           << DST::integral(sinx, cosx, txa.data(), 2*N) << "\n";
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
}
