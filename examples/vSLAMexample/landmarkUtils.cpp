#include "landmarkUtils.h"
#include <fstream>
#include <cstdio>

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
bool readLandMarks(const char* landmarkFile, std::map<int, Point3>& landmarks)
{
    ifstream file(landmarkFile);
    if (!file) {
        cout << "Cannot read landmark file: " << landmarkFile << endl;
        exit(0);
    }

    int num;
    file >> num;
    landmarks.clear();
    for (int i = 0; i<num; i++)
    {
        int color_id;
        float x, y, z;
        file >> color_id >> x >> y >> z;
        landmarks[color_id] = Point3(x, y, z);
    }

    file.close();
    return true;
}

/* ************************************************************************* */
/**
  * Read pose from file, output by Panda3D.
  * Warning: row major!!!
  */
gtsam::Pose3 readPose(const char* Fn)
{
    ifstream poseFile(Fn);
    if (!poseFile)
    {
        cout << "Cannot read pose file: " << Fn << endl;
        exit(0);
    }

    double v[16];
    for (int i = 0; i<16; i++)
        poseFile >> v[i];
    poseFile.close();

    // Because panda3d's camera is z-up, y-view,
    // we swap z and y to have y-up, z-view, then negate z to stick with the right-hand rule
    //... similar to OpenGL's camera
    for (int i = 0; i<3; i++)
    {
        float t = v[4+i];
        v[4+i] = v[8+i];
        v[8+i] = -t;
    }

    ::Vector vec = Vector_(16, v);

    Matrix T = Matrix_(4,4, vec);   // column order !!!

    Pose3 pose(T);
    return pose;
}
/* ************************************************************************* */
gtsam::Pose3 readPose(const char* poseFn_pre, const char* poseFn_suf, int poseId)
{
    char poseFn[128];
    sprintf(poseFn, "%s%d%s", poseFn_pre, poseId, poseFn_suf);
    return readPose(poseFn);
}

/* ************************************************************************* */
gtsam::Cal3_S2 readCalibData(const char* calibFn)
{
    ifstream calibFile(calibFn);
    if (!calibFile)
    {
        cout << "Cannot read calib file: " << calibFn << endl;
        exit(0);
    }
    int imX, imY;
    float fx, fy, ox, oy;
    calibFile >> imX >> imY >> fx >> fy >> ox >> oy;
    calibFile.close();

    Cal3_S2 K(fx, fy, 0, ox, oy);   // skew factor = 0
    return K;
}
/* ************************************************************************* */
std::vector<Feature2D> readFeatures(const char* filename)
{
    ifstream file(filename);
    if (!file)
    {
        cout << "Cannot read feature file: " << filename<< endl;
        exit(0);
    }

    int numFeatures;
    file >> numFeatures ;

    std::vector<Feature2D> vFeatures_;
    for (size_t i = 0; i < numFeatures; i++)
    {
        int id; double x, y;
        file >> id >> x >> y;
        vFeatures_.push_back(Feature2D(id, Point2(x, y)));
    }

    file.close();
    return vFeatures_;
}
/* ************************************************************************* */
std::vector<Feature2D> readFeatures(const char* featFn_pre, const char* featFn_suf, int imageId)
{
    char featFn[128];
    sprintf(featFn, "%s%d%s", featFn_pre, imageId, featFn_suf);
    return readFeatures(featFn);
}
