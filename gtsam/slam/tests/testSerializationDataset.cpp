/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationDataset.cpp
 * @brief serialization tests for dataset.cpp
 * @author Ayush Baid
 * @date Jan 1, 2021
 */

#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/dataset.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* ************************************************************************* */
TEST(dataSet, sfmDataSerialization) {
  // Test the serialization of SfmData
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData mydata = SfmData::FromBalFile(filename);

  // round-trip equality check on serialization and subsequent deserialization
  EXPECT(equalsObj(mydata));
  EXPECT(equalsXML(mydata));
  EXPECT(equalsBinary(mydata));
}

/* ************************************************************************* */
TEST(dataSet, sfmTrackSerialization) {
  // Test the serialization of SfmTrack
  const string filename = findExampleDataFile("dubrovnik-3-7-pre");
  SfmData mydata = SfmData::FromBalFile(filename);

  SfmTrack track = mydata.track(0);

  // round-trip equality check on serialization and subsequent deserialization
  EXPECT(equalsObj(track));
  EXPECT(equalsXML(track));
  EXPECT(equalsBinary(track));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
