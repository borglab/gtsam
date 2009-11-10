/**
 * @file   VSLAMConfig.cpp
 * @brief  The Config
 * @author Alireza Fathi
 * @author Carlos Nieto
 */

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "VSLAMConfig.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

#define SIGMA 1.0

namespace gtsam{

/* ************************************************************************* */
// add a delta to a DCVectorConfig
// Very ugly: there are 3D vectors for points, and 12D vectors for poses
// We check the dimension and do the right thing for each.
VectorConfig DCVectorConfig::operator+(const VectorConfig & delta) const {
  DCVectorConfig result;
  string j; Vector v;
  FOREACH_PAIR(j, v, values) {
    if (v.size()==3) {
      Point3 basePoint(v);
      Point3 newPoint(basePoint.exmap( delta[j] ));
      result.insert( j, newPoint.vector() );
    } else {
      Pose3 basePose(v);
      Pose3 newPose(basePose.exmap( delta[j] ));
      result.insert( j, newPose.vector() );
    }
  }
  return result;
}

/* ************************************************************************* */
void DCVectorConfig::operator+=(const VectorConfig & delta) {
  for (iterator it = values.begin(); it!=values.end(); it++) {
    string j = it->first;
    Vector &v = it->second; // reference !
    // code below changes the reference in the config in-place !!!
    if (v.size()==3) {
      Point3 basePoint(v);
      Point3 newPoint(basePoint.exmap( delta[j] ));
      v = newPoint.vector();
    } else {
      Pose3 basePose(v);
      Pose3 newPose(basePose.exmap( delta[j] ));
      v = newPose.vector();
    }
  }
}

/* ************************************************************************* */
VSLAMConfig::VSLAMConfig(VectorConfig & fgconfig) {
  landmarkPoints.clear();
  cameraPoses.clear();

  for(map<std::string, Vector>::const_iterator it = fgconfig.begin(); it != fgconfig.end(); it++)
  {
    string temp = (*it).first;
    if(temp[0] == 'x')
    {
      int cameraNumber = atoi(temp.substr(1,temp.size()-1).c_str());
      Pose3 cameraPose(Pose3((*it).second));
      addCameraPose(cameraNumber, cameraPose);
    }
    if(temp[0] == 'l')
    {
      int landmarkNumber = atoi(temp.substr(1,temp.size()-1).c_str());
      Pose3 markerPose(Pose3((*it).second));
      Point3 landmarkPoint = markerPose.translation();
      addLandmarkPoint(landmarkNumber, landmarkPoint);
    }
  }

}


// Exponential map
// TODO: FD: I think this is horrible
VSLAMConfig VSLAMConfig::exmap(const VectorConfig & delta) const {

	VSLAMConfig newConfig;

	for (map<string, Vector>::const_iterator it = delta.begin(); it
			!= delta.end(); it++) {
		string key = it->first;
		if (key[0] == 'x') {
			int cameraNumber = atoi(key.substr(1, key.size() - 1).c_str());
			Pose3 basePose = cameraPose(cameraNumber);
			newConfig.addCameraPose(cameraNumber, basePose.exmap(it->second));
		}
		if (key[0] == 'l') {
			int landmarkNumber = atoi(key.substr(1, key.size() - 1).c_str());
			Point3 basePoint = landmarkPoint(landmarkNumber);
			newConfig.addLandmarkPoint(landmarkNumber, basePoint.exmap(it->second));
		}
	}

	return newConfig;
}

// Not used!
/* ************************************************************************* *
void VSLAMConfig::load(std::string& path, int num_of_frames)
{
  char putatives[200];
  putatives[0] = 0;
  sprintf(putatives, "%s/putatives.txt", path.c_str());

  char poseR[200];
  poseR[0] = 0;
  sprintf(poseR, "%s/pose_R_info.txt", path.c_str());

  char poseXYZ[200];
  poseXYZ[0] = 0;
  sprintf(poseXYZ, "%s/pose_xyz_info.txt", path.c_str());

  std::ifstream putatives_In(path.c_str(), ios::in);
  std::ifstream poseR_In(path.c_str(), ios::in);
  std::ifstream poseXYZ_In(path.c_str(), ios::in);

  int frameNumber;
  int landmarkIndex;

   double landmarkX;
   double landmarkY;
   double landmarkZ;

   double uLs;
   double vLs;

   int cameraIndex;

   double cameraX;
   double cameraY;
   double cameraZ;

   double pitch;
   double yaw;
   double roll;


   if(putatives_In)
      putatives_In >> frameNumber >> landmarkIndex >> uLs >> vLs >> landmarkX >> landmarkY >> landmarkZ;
    else
    {
      printf("Unable to load values from putatives\n");
      exit(0);
    }

   if(poseR_In)
      poseR_In >> yaw >> pitch >> roll;
    else
    {
      printf("Unable to load values from poseR\n");
      exit(0);
    }

   if(poseXYZ_In)
      poseXYZ_In >> cameraX >> cameraY >> cameraZ;
    else
    {
      printf("Unable to load values from poseXYZ\n");
      exit(0);
    }

    char lname [50];
    sprintf (lname, "l%d", frameNumber);


    landmarkPoints[landmarkIndex] = Point3(landmarkX, landmarkY, landmarkZ);

    Rot3 R = rodriguez(yaw,pitch,roll);
    Point3 c (cameraX, cameraY, cameraZ);

    cameraPoses[cameraIndex]= Pose3(R,c);

    putatives_In.close();
    poseR_In.close();
    poseXYZ_In.close();
}

/* ************************************************************************* */
void VSLAMConfig::flush(int referenceMarker, const std::string& path)
{



}


/* ************************************************************************* */
void VSLAMConfig::print(const std::string& s) const
{
  printf("%s:\n", s.c_str());
  printf("Camera Poses:\n");
  for(PoseMap::const_iterator it = cameraIteratorBegin(); it != cameraIteratorEnd(); it++)
  {
    printf("x%d:\n", it->first);
    it->second.print();
  }
  printf("Landmark Points:\n");
  for(PointMap::const_iterator it = landmarkIteratorBegin(); it != landmarkIteratorEnd(); it++)
  {
    printf("l%d:\n", (*it).first);
    (*it).second.print();
  }
}

/* ************************************************************************* */
bool VSLAMConfig::equals(const VSLAMConfig& c) {
  for(PoseMap::const_iterator it = cameraIteratorBegin(); it != cameraIteratorEnd(); it++)
  {
    if(!c.cameraPoseExists(it->first))
    {
      printf("camera pose %d didn't exist in that\n", it->first);
      goto fail;
    }
    if(!assert_equal(it->second, c.cameraPose(it->first), 1e-6))
      goto fail;
  }

  for(PointMap::const_iterator it = landmarkIteratorBegin(); it != landmarkIteratorEnd(); it++)
  {
    if(!c.landmarkPointExists(it->first))
    {
      printf("landmark point %d didn't exist in that\n", it->first);
      goto fail;
    }
    if(!assert_equal(it->second, c.landmarkPoint(it->first), 1e-6))
      goto fail;
  }

  return true;

 fail:
  print("this");
  c.print("that");
  return false;
}

/* ************************************************************************* */
void VSLAMConfig::addCameraPose(const int i, Pose3 cp)
{
  pair<int, Pose3> camera;
  camera.first = i;
  camera.second = cp;
  cameraPoses.insert(camera);
}
/* ************************************************************************* */
void VSLAMConfig::addLandmarkPoint(const int i, Point3 lp)
{
  pair<int, Point3> landmark;
  landmark.first = i;
  landmark.second = lp;
  landmarkPoints.insert(landmark);
}
/* ************************************************************************* */
void VSLAMConfig::removeCameraPose(const int i)
{
  if(cameraPoseExists(i))
    cameraPoses.erase(i);
}
/* ************************************************************************* */
void VSLAMConfig::removeLandmarkPose(const int i)
{
  if(landmarkPointExists(i))
    landmarkPoints.erase(i);
}
/* ************************************************************************* */
DCVectorConfig VSLAMConfig::getVectorConfig() const
{
  DCVectorConfig cfg;
  char buffer[100];
  string stbuf;
  int ln; Point3 landmarkPoint;
  FOREACH_PAIR(ln, landmarkPoint, landmarkPoints) {
    buffer[0] = 0;
    sprintf(buffer, "l%d", ln);
    stbuf = string(buffer);
    cfg.insert(stbuf, landmarkPoint.vector()); // 3D for points
  }
  int cn; Pose3 cameraPose;
  FOREACH_PAIR(cn, cameraPose, cameraPoses) {
    buffer[0] = 0;
    sprintf(buffer, "x%d", cn);
    stbuf = string(buffer);
    cfg.insert(stbuf, cameraPose.vector()); // 12D for poses
  }
  return cfg;
}

} // namespace gtsam

