/**
 * @file    vSLAMexample.cpp
 * @brief   An vSLAM example for synthesis sequence
 * single camera
 * @author  Duy-Nguyen
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <boost/shared_ptr.hpp>
using namespace boost;

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/PriorFactor.h>

#include "landmarkUtils.h"
#include "Feature2D.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;
using namespace boost;
typedef NonlinearOptimizer<Graph,Values> Optimizer;

/* ************************************************************************* */
#define CALIB_FILE      "Data/calib.txt"
#define LANDMARKS_FILE  "Data/landmarks.txt"

#define POSEFN_PREFIX   "Data/ttpy"
#define POSEFN_SUFFIX   ".pose"
#define FEATFN_PREFIX   "Data/ttpy"
#define FEATFN_SUFFIX   ".feat"

#define NUM_IMAGES 10
const int ImageIds[NUM_IMAGES] = {10,20,30,40,50,60,70,80,90,100};

// Store groundtruth values
map<int, Point3> g_landmarks;
vector<Pose3> g_poses;

/* ************************************************************************* */
/**
  * Setup vSLAM graph
  * by adding and associating 2D features (measurements) detected in each image
  * with their corresponding landmarks.
  */
Graph setupGraph()
{
    Graph g;
    shared_ptrK sK(new Cal3_S2(readCalibData(CALIB_FILE)));
    sK->print("Calibration: ");
		SharedGaussian sigma(noiseModel::Isotropic::Sigma(2,5.0f));

    for (size_t i= 0; i<NUM_IMAGES; i++)
    {
        std::vector<Feature2D> features = readFeatures(FEATFN_PREFIX, FEATFN_SUFFIX, ImageIds[i]);
        for (size_t j = 0; j<features.size(); j++)
            g.addMeasurement(features[j].m_p, sigma, i, features[j].m_id, sK);
    }

    return g;
}

/* ************************************************************************* */
/**
  * Read initial values.
  * Note: These are ground-truth values, but we just use them as initial estimates.
  */
Values initializeValues()
{
    Values initValues;

    // Initialize landmarks 3D positions.
    for (map<int, Point3>::iterator lmit = g_landmarks.begin(); lmit != g_landmarks.end(); lmit++)
        initValues.insert( lmit->first, lmit->second );

    // Initialize camera poses.
    for (int i = 0; i<NUM_IMAGES; i++)
        initValues.insert(i, g_poses[i]);

    return initValues;
}

/* ************************************************************************* */
int main()
{
    shared_ptr<Graph> graph(new Graph(setupGraph()));

    // Read groundtruth landmarks' positions and poses. These will also be used later as intial estimates.
    readLandMarks(LANDMARKS_FILE, g_landmarks);
    for (int i = 0; i<NUM_IMAGES; i++)
    {
        Pose3 pose = readPose(POSEFN_PREFIX, POSEFN_SUFFIX, ImageIds[i]) ;
        g_poses.push_back( pose );
    }

    // Create an initial Values structure using groundtruth as the initial estimates
    boost::shared_ptr<Values> initialValues(new Values(initializeValues()));

    // Add hard constraint on the first pose, used as fixed prior.
		graph->addPoseConstraint(0, g_poses[0]);

    // Create an ordering of the variables
    shared_ptr<Ordering> ordering(new Ordering);
    char name[4];
    for (size_t i = 0; i<g_landmarks.size(); i++)
    {
        sprintf(name, "l%d", i);    // "li"
        *ordering += name;
    }

    for (size_t i = 0; i<NUM_IMAGES; i++)
    {
        sprintf(name, "x%d", i);    // "xj"
        *ordering += name;
    }

    // Create an optimizer and check its error
    // We expect the initial to be zero because Values is the ground truth
        Optimizer::shared_solver solver(new Optimizer::solver(ordering));
    Optimizer optimizer(graph, initialValues, solver);
    cout << "Initial error: " << optimizer.error() << endl;
    optimizer.config()->print("Initial estimates: ");

    // Optimize the graph.
		Optimizer::verbosityLevel verborsity = Optimizer::ERROR;
		Optimizer optimResult = optimizer.levenbergMarquardt(1e-5, 1e-5, verborsity);
    optimResult.config()->print("After optimization: ");

}
/* ************************************************************************* */

