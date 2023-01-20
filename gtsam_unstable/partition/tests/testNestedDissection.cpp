/*
 * testNestedDissection.cpp
 *
 *   Created on: Nov 29, 2010
 *       Author: nikai
 *  Description: unit tests for NestedDissection
 */

#include <boost/make_shared.hpp>
#include <CppUnitLite/TestHarness.h>

#include "SubmapPlanarSLAM.h"
#include "SubmapVisualSLAM.h"
#include "SubmapExamples.h"
#include "SubmapExamples3D.h"
#include "GenericGraph.h"
#include "NonlinearTSAM.h"
#include "partition/NestedDissection-inl.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::partition;

/* ************************************************************************* */
// x1 - x2
//  \  /
//   l1
TEST ( NestedDissection, oneIsland )
{
  using namespace submapPlanarSLAM;
  typedef TSAM2D::SubNLG SubNLG;
  Graph fg;
  fg.addOdometry(1, 2, Pose2(), odoNoise);
  fg.addBearingRange(1, 1, Rot2(), 0., bearingRangeNoise);
  fg.addBearingRange(2, 1, Rot2(), 0., bearingRangeNoise);
  fg.addPoseConstraint(1, Pose2());

  Ordering ordering; ordering += x1, x2, l1;

  int numNodeStopPartition = 1e3;
  int minNodesPerMap = 1e3;
  NestedDissection<Graph, SubNLG, GenericGraph2D> nd(fg, ordering, numNodeStopPartition, minNodesPerMap);
  LONGS_EQUAL(4, nd.root()->size());
  LONGS_EQUAL(3, nd.root()->frontal().size());
  LONGS_EQUAL(0, nd.root()->children().size());
}

/* ************************************************************************* */
// x1\  /x4
// |  x3  |
// x2/  \x5
TEST ( NestedDissection, TwoIslands )
{
  using namespace submapPlanarSLAM;
  typedef TSAM2D::SubNLG SubNLG;
  Graph fg;
  fg.addOdometry(1, 2, Pose2(), odoNoise);
  fg.addOdometry(1, 3, Pose2(), odoNoise);
  fg.addOdometry(2, 3, Pose2(), odoNoise);
  fg.addOdometry(3, 4, Pose2(), odoNoise);
  fg.addOdometry(4, 5, Pose2(), odoNoise);
  fg.addOdometry(3, 5, Pose2(), odoNoise);
  fg.addPoseConstraint(1, Pose2());
  fg.addPoseConstraint(4, Pose2());
  Ordering ordering; ordering += x1, x2, x3, x4, x5;

  int numNodeStopPartition = 2;
  int minNodesPerMap = 1;
  NestedDissection<Graph, SubNLG, GenericGraph2D> nd(fg, ordering, numNodeStopPartition, minNodesPerMap);
  // root submap
  LONGS_EQUAL(0, nd.root()->size());
  LONGS_EQUAL(1, nd.root()->frontal().size());
  LONGS_EQUAL(0, nd.root()->separator().size());
  LONGS_EQUAL(2, nd.root()->children().size()); // 2 leaf submaps

  // the 1st submap
  LONGS_EQUAL(2, nd.root()->children()[0]->frontal().size());
  LONGS_EQUAL(4, nd.root()->children()[0]->size());

  // the 2nd submap
  LONGS_EQUAL(2, nd.root()->children()[1]->frontal().size());
  LONGS_EQUAL(4, nd.root()->children()[1]->size());
}

/* ************************************************************************* */
// x1\  /x4
//    x3
// x2/  \x5
TEST ( NestedDissection, FourIslands )
{
  using namespace submapPlanarSLAM;
  typedef TSAM2D::SubNLG SubNLG;
  Graph fg;
  fg.addOdometry(1, 3, Pose2(), odoNoise);
  fg.addOdometry(2, 3, Pose2(), odoNoise);
  fg.addOdometry(3, 4, Pose2(), odoNoise);
  fg.addOdometry(3, 5, Pose2(), odoNoise);
  fg.addPoseConstraint(1, Pose2());
  fg.addPoseConstraint(4, Pose2());
  Ordering ordering; ordering += x1, x2, x3, x4, x5;

  int numNodeStopPartition = 2;
  int minNodesPerMap = 1;
  NestedDissection<Graph, SubNLG, GenericGraph2D> nd(fg, ordering, numNodeStopPartition, minNodesPerMap);
  LONGS_EQUAL(0, nd.root()->size());
  LONGS_EQUAL(1, nd.root()->frontal().size());
  LONGS_EQUAL(0, nd.root()->separator().size());
  LONGS_EQUAL(4, nd.root()->children().size()); // 4 leaf submaps

  // the 1st submap
  LONGS_EQUAL(1, nd.root()->children()[0]->frontal().size());
  LONGS_EQUAL(2, nd.root()->children()[0]->size());

  // the 2nd submap
  LONGS_EQUAL(1, nd.root()->children()[1]->frontal().size());
  LONGS_EQUAL(2, nd.root()->children()[1]->size());

  // the 3rd submap
  LONGS_EQUAL(1, nd.root()->children()[2]->frontal().size());
  LONGS_EQUAL(1, nd.root()->children()[2]->size());

  // the 4th submap
  LONGS_EQUAL(1, nd.root()->children()[3]->frontal().size());
  LONGS_EQUAL(1, nd.root()->children()[3]->size());
}

/* ************************************************************************* */
// x1\  /x3
//  | x2  |
// l6/  \x4
//  |
// x5
TEST ( NestedDissection, weekLinks )
{
  using namespace submapPlanarSLAM;
  typedef TSAM2D::SubNLG SubNLG;
  Graph fg;
  fg.addOdometry(1, 2, Pose2(), odoNoise);
  fg.addOdometry(2, 3, Pose2(), odoNoise);
  fg.addOdometry(2, 4, Pose2(), odoNoise);
  fg.addOdometry(3, 4, Pose2(), odoNoise);
  fg.addBearingRange(1, 6, Rot2(), 0., bearingRangeNoise);
  fg.addBearingRange(2, 6, Rot2(), 0., bearingRangeNoise);
  fg.addBearingRange(5, 6, Rot2(), 0., bearingRangeNoise);
  fg.addPoseConstraint(1, Pose2());
  fg.addPoseConstraint(4, Pose2());
  fg.addPoseConstraint(5, Pose2());
  Ordering ordering; ordering += x1, x2, x3, x4, x5, l6;

  int numNodeStopPartition = 2;
  int minNodesPerMap = 1;
  NestedDissection<Graph, SubNLG, GenericGraph2D> nd(fg, ordering, numNodeStopPartition, minNodesPerMap);
  LONGS_EQUAL(0, nd.root()->size()); // one weeklink
  LONGS_EQUAL(1, nd.root()->frontal().size());
  LONGS_EQUAL(0, nd.root()->separator().size());
  LONGS_EQUAL(3, nd.root()->children().size()); // 4 leaf submaps
  LONGS_EQUAL(1, nd.root()->weeklinks().size());

  // the 1st submap
  LONGS_EQUAL(2, nd.root()->children()[0]->frontal().size()); // x3 and x4
  LONGS_EQUAL(4, nd.root()->children()[0]->size());

  // the 2nd submap
  LONGS_EQUAL(2, nd.root()->children()[1]->frontal().size()); // x1 and l6
  LONGS_EQUAL(4, nd.root()->children()[1]->size());
  //
  // the 3rd submap
  LONGS_EQUAL(1, nd.root()->children()[2]->frontal().size()); // x5
  LONGS_EQUAL(1, nd.root()->children()[2]->size());
}

/* ************************************************************************* */
/**
 *  l1   l2   l3
 *   | X  | X |
 *  x0 - x1 - x2
 *   | X  | X |
 *  l4   l5   l6
 */
TEST ( NestedDissection, manual_cuts )
{
  using namespace submapPlanarSLAM;
  typedef partition::Cuts Cuts;
  typedef TSAM2D::SubNLG SubNLG;
  typedef partition::PartitionTable PartitionTable;
  Graph fg;
  fg.addOdometry(x0, x1, Pose2(1.0, 0, 0), odoNoise);
  fg.addOdometry(x1, x2, Pose2(1.0, 0, 0), odoNoise);

  fg.addBearingRange(x0, l1, Rot2::fromAngle( M_PI_2), 1,       bearingRangeNoise);
  fg.addBearingRange(x0, l4, Rot2::fromAngle(-M_PI_2), 1,       bearingRangeNoise);
  fg.addBearingRange(x0, l2, Rot2::fromAngle( M_PI_4), sqrt(2), bearingRangeNoise);
  fg.addBearingRange(x0, l5, Rot2::fromAngle(-M_PI_4), sqrt(2), bearingRangeNoise);

  fg.addBearingRange(x1, l1, Rot2::fromAngle( M_PI_4 * 3), sqrt(2), bearingRangeNoise);
  fg.addBearingRange(x1, l2, Rot2::fromAngle( M_PI_2),     1,       bearingRangeNoise);
  fg.addBearingRange(x1, l3, Rot2::fromAngle( M_PI_4),     sqrt(2), bearingRangeNoise);
  fg.addBearingRange(x1, l4, Rot2::fromAngle(-M_PI_4 * 3), sqrt(2), bearingRangeNoise);
  fg.addBearingRange(x1, l5, Rot2::fromAngle( M_PI_2),     1,       bearingRangeNoise);
  fg.addBearingRange(x1, l6, Rot2::fromAngle(-M_PI_4),     sqrt(2), bearingRangeNoise);

  fg.addBearingRange(x2, l2, Rot2::fromAngle( M_PI_4 * 3), sqrt(2), bearingRangeNoise);
  fg.addBearingRange(x2, l5, Rot2::fromAngle(-M_PI_4 * 3), sqrt(2), bearingRangeNoise);
  fg.addBearingRange(x2, l3, Rot2::fromAngle( M_PI_2),     1,       bearingRangeNoise);
  fg.addBearingRange(x2, l6, Rot2::fromAngle(-M_PI_2),     1,       bearingRangeNoise);

  fg.addPrior(x0, Pose2(0.1, 0, 0), priorNoise);

  // generate ordering
  Ordering ordering; ordering += x0, x1, x2, l1, l2, l3, l4, l5, l6;

  // define cuts
  boost::shared_ptr<Cuts> cuts(new Cuts());
  cuts->partitionTable = PartitionTable(9, -1); PartitionTable* p = &cuts->partitionTable;
  //x0        x1         x2         l1         l2         l3         l4         l5         l6
  (*p)[0]=1; (*p)[1]=0; (*p)[2]=2; (*p)[3]=1; (*p)[4]=0; (*p)[5]=2; (*p)[6]=1; (*p)[7]=0; (*p)[8]=2;

  cuts->children.push_back(boost::shared_ptr<Cuts>(new Cuts()));
  cuts->children[0]->partitionTable = PartitionTable(9, -1); p = &cuts->children[0]->partitionTable;
  //x0        x1          x2          l1         l2          l3          l4         l5          l6
  (*p)[0]=0; (*p)[1]=-1; (*p)[2]=-1; (*p)[3]=1; (*p)[4]=-1; (*p)[5]=-1; (*p)[6]=2; (*p)[7]=-1; (*p)[8]=-1;

  cuts->children.push_back(boost::shared_ptr<Cuts>(new Cuts()));
  cuts->children[1]->partitionTable = PartitionTable(9, -1); p = &cuts->children[1]->partitionTable;
  //x0         x1          x2         l1          l2          l3         l4          l5          l6
  (*p)[0]=-1; (*p)[1]=-1; (*p)[2]=0; (*p)[3]=-1; (*p)[4]=-1; (*p)[5]=1; (*p)[6]=-1; (*p)[7]=-1; (*p)[8]=2;


  // nested dissection
  NestedDissection<Graph, SubNLG, GenericGraph2D> nd(fg, ordering, cuts);
  LONGS_EQUAL(2, nd.root()->size());
  LONGS_EQUAL(3, nd.root()->frontal().size());
  LONGS_EQUAL(0, nd.root()->separator().size());
  LONGS_EQUAL(2, nd.root()->children().size()); // 2 leaf submaps
  LONGS_EQUAL(0, nd.root()->weeklinks().size());

  // the 1st submap
  LONGS_EQUAL(1, nd.root()->children()[0]->frontal().size()); // x0
  LONGS_EQUAL(4, nd.root()->children()[0]->size());
  LONGS_EQUAL(2, nd.root()->children()[0]->children().size());

  // the 1-1st submap
  LONGS_EQUAL(1, nd.root()->children()[0]->children()[0]->frontal().size()); // l1
  LONGS_EQUAL(2, nd.root()->children()[0]->children()[0]->size());

  // the 1-2nd submap
  LONGS_EQUAL(1, nd.root()->children()[0]->children()[1]->frontal().size()); // l4
  LONGS_EQUAL(2, nd.root()->children()[0]->children()[1]->size());

  // the 2nd submap
  LONGS_EQUAL(1, nd.root()->children()[1]->frontal().size()); // x2
  LONGS_EQUAL(3, nd.root()->children()[1]->size());
  LONGS_EQUAL(2, nd.root()->children()[1]->children().size());

  // the 2-1st submap
  LONGS_EQUAL(1, nd.root()->children()[1]->children()[0]->frontal().size()); // l3
  LONGS_EQUAL(2, nd.root()->children()[1]->children()[0]->size());

  // the 2-2nd submap
  LONGS_EQUAL(1, nd.root()->children()[1]->children()[1]->frontal().size()); // l6
  LONGS_EQUAL(2, nd.root()->children()[1]->children()[1]->size());

}

/* ************************************************************************* */
//   l1-l8   l9-16    l17-124
//    / |    /    \    | \
// x0  x1             x2  x3
TEST( NestedDissection, Graph3D) {
  using namespace gtsam::submapVisualSLAM;
  typedef TSAM3D::SubNLG SubNLG;
  typedef partition::PartitionTable PartitionTable;
  vector<GeneralCamera> cameras;
  cameras.push_back(GeneralCamera(Pose3(Rot3(), Point3(-2., 0., 0.))));
  cameras.push_back(GeneralCamera(Pose3(Rot3(), Point3(-1., 0., 0.))));
  cameras.push_back(GeneralCamera(Pose3(Rot3(), Point3( 1., 0., 0.))));
  cameras.push_back(GeneralCamera(Pose3(Rot3(), Point3( 2., 0., 0.))));

  vector<Point3> points;
  for (int cube_index = 0; cube_index <= 3; cube_index++) {
    Point3 center((cube_index-1) * 3, 0.5, 10.);
    points.push_back(center + Point3(-0.5, -0.5, -0.5));
    points.push_back(center + Point3(-0.5,  0.5, -0.5));
    points.push_back(center + Point3( 0.5,  0.5, -0.5));
    points.push_back(center + Point3( 0.5, -0.5, -0.5));
    points.push_back(center + Point3(-0.5, -0.5,  0.5));
    points.push_back(center + Point3(-0.5,  0.5,  0.5));
    points.push_back(center + Point3( 0.5,  0.5,  0.5));
    points.push_back(center + Point3( 0.5,  0.5,  0.5));
  }

  Graph graph;
  SharedDiagonal measurementNoise(gtsam::Vector_(2, 1., 1.));
  SharedDiagonal measurementZeroNoise(gtsam::Vector_(2, 0., 0.));
  for (int j=1; j<=8; j++)
    graph.addMeasurement(0, j, cameras[0].project(points[j-1]).expmap(measurementZeroNoise->sample()), measurementNoise);
  for (int j=1; j<=16; j++)
    graph.addMeasurement(1, j, cameras[1].project(points[j-1]).expmap(measurementZeroNoise->sample()), measurementNoise);
  for (int j=9; j<=24; j++)
    graph.addMeasurement(2, j, cameras[2].project(points[j-1]).expmap(measurementZeroNoise->sample()), measurementNoise);
  for (int j=17; j<=24; j++)
    graph.addMeasurement(3, j, cameras[3].project(points[j-1]).expmap(measurementZeroNoise->sample()), measurementNoise);

  // make an easy ordering
  Ordering ordering; ordering += x0, x1, x2, x3;
  for (int j=1; j<=24; j++)
    ordering += Symbol('l', j);

  // nested dissection
  const int numNodeStopPartition = 10;
  const int minNodesPerMap = 5;
  NestedDissection<Graph, SubNLG, GenericGraph3D> nd(graph, ordering, numNodeStopPartition, minNodesPerMap);

  LONGS_EQUAL(0, nd.root()->size());
  LONGS_EQUAL(8, nd.root()->frontal().size()); // l9-l16
  LONGS_EQUAL(0, nd.root()->separator().size());
  LONGS_EQUAL(2, nd.root()->children().size()); // 2 leaf submaps
  LONGS_EQUAL(0, nd.root()->weeklinks().size());

  // the 1st submap
  LONGS_EQUAL(10, nd.root()->children()[0]->frontal().size()); // x0, x1, l1-l8
  LONGS_EQUAL(24, nd.root()->children()[0]->size()); // 8 + 16
  LONGS_EQUAL(0, nd.root()->children()[0]->children().size());

  // the 2nd submap
  LONGS_EQUAL(10, nd.root()->children()[1]->frontal().size()); // x2, x3, l1-l8
  LONGS_EQUAL(24, nd.root()->children()[1]->size()); // 16 + 8
  LONGS_EQUAL(0, nd.root()->children()[1]->children().size());
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
