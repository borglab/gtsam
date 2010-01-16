/*
 * testNoiseModel.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: richard
 */

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <iostream>
#include "NoiseModel.h"

using namespace std;
using namespace boost;
using namespace gtsam;

class TakesNoiseModel {
public:
  NoiseModel noiseModel_;
public:
  //template<class N>
  TakesNoiseModel(const NoiseModel& noiseModel): noiseModel_(noiseModel) {}
};


TEST(NoiseModel, sharedptr)
{
  TakesNoiseModel tnm1(Sigma(1.0));
  cout << endl;
  TakesNoiseModel tnm2(tnm1.noiseModel_);

  if(dynamic_pointer_cast<Sigma>(tnm1.noiseModel_))
    cout << "tnm1 has a Sigma!" << endl;
  if(dynamic_pointer_cast<Variance>(tnm1.noiseModel_))
    cout << "tnm1 has a Variance!" << endl;
  if(dynamic_pointer_cast<Isotropic>(tnm1.noiseModel_))
    cout << "tnm1 has an Isotropic!" << endl;
  if(dynamic_pointer_cast<NoiseModelBase>(tnm1.noiseModel_))
    cout << "tnm1 has a NoiseModelBase!" << endl;

  if(dynamic_pointer_cast<Sigma>(tnm2.noiseModel_))
    cout << "tnm2 has a Sigma!" << endl;
  if(dynamic_pointer_cast<Variance>(tnm2.noiseModel_))
    cout << "tnm2 has a Variance!" << endl;
  if(dynamic_pointer_cast<Isotropic>(tnm2.noiseModel_))
    cout << "tnm2 has an Isotropic!" << endl;
  if(dynamic_pointer_cast<NoiseModelBase>(tnm2.noiseModel_))
    cout << "tnm2 has a NoiseModelBase!" << endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
