How To Use MH-iSAM2
===================

Assuming that the front-end of a SLAM system can detect ambiguity and get individual measurements of each mode, we can construct a multi-mode factor (MMF) with corresponding type to model it. The MMFs can be added into a multi-hypothesis factor graph (MHFG) with traditional single-mode factors (SMF) to estimate multiple possible solutions of the multi-hypothesis variables (MHV). The optimization algorithm is based on multi-hypothesis Bayes tree (MHBT) and Hypo-tree. Please refer to [MH-iSAM2]() for more information about the types of the MMF, the optimization algorithm, and the applied data structures.

Following are some implementation guidances when using MH-iSAM2. Please refer to the examples as well for better understanding.


Setup MH-iSAM2
--------------

Include necessary headers in MH-iSAM2_lib:

```cpp
// Always required
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// To use symbol_shorthand for Keys of MHVs
#include <gtsam/inference/Symbol.h>

// Replace T.h with the name of the files that define the types of variables that will be used in the program, e.g.: Pose3.h, Point3.h, ...
#include <gtsam/geometry/T.h> 

// Replace *Factor.h with the name of the files that define the types of basic SMFs and MMFs that will be used in the program, e.g.: PriorFactor.h, BetweenFactor.h, ...
#include <gtsam/slam/*Factor.h>
```

To define your own SMF/MMF, please refer to:

i. MHPriorFactor, which can serve as a SMF, Type #1 MMF, Type #3 MMF, or Type #1 + #3 MMF and is defined in:
```
gtsam/slam/PriorFactor.h 
```

ii. MHBetweenFactor, which can serve as a SMF, Type #1 MMF, Type #3 MMF, or Type #1 + #3 MMF and is defined in:
```
gtsam/slam/BetweenFactor.h
```

iii. MH_Pose2_Point2_Factor, which can serve as a SMF, Type #2 MMF, Type #3 MMF, or Type #2 + #3 MMF and is defined in:
```
gtsam/slam/Pose2_Point2_Factor.h
```

Setup environment:

```cpp
using namespace gtsam;
using symbol_shorthand::X; //the 1-to-1 Keys of MHVs
```

Setup MH-iSAM2 parameters and necessary components:

```cpp
// Parameters for original iSAM2 algorithm in GTSAM are also used in MH-iSAM2
ISAM2Params params;
params.optimizationParams = ISAM2GaussNewtonParams(0.0);
params.relinearizeThreshold = 0.01;
params.relinearizeSkip = 1;

// Additional parameters for MH-iSAM2
MHParams mh_params(n_desired, n_limit);

// Initialize with both types of parameters 
MHISAM2* mh_isam2 = new MHISAM2(params, mh_params);

// Subgraph to record new SMFs and MMFs that should be added into the MHBT in each iteration
NonlinearFactorGraph* mh_graph = new NonlinearFactorGraph();

// Initial values and output estimations of MHVs 
Values mh_init_values;
Values mh_results;
```

Notice that the old containers NonlinearFactorGraph and Values are extended to take SMF, MMF, and MHV. However, they cannot be used together with the original *Factor and Value defined in GTSAM.


Construct SMF and MMFs / Initialize MHVs
----------------------------------------

In general, there are 4 different cases:

Case 1. SMF && not creating a new loop in the factor graph (e.g.: odometry between an old pose and a new pose)

```cpp
// Add a new MHV
mh_init_values.mhInsert( MHGenericValue<T>(std::vector<T> predict_value_arr) ); 
    //T: the type of variable to estimate, e.g.: Pose3 
    //predict_value_arr.size() should be 1

// Add a new SMF
mh_graph->add( MHNF<T>(Keys, std::vector<T> measured_arr) ); 
    //MHNF: a factor that can serve as a SMF or MMF (derive from MHNoiseModelFactor), e.g.: MHBetweenFactor<Pose3> 
    //Keys: see examples 
    //measured_arr.size() should be 1
```

Case 2. MMF && not creating a new loop in the factor graph (e.g.: ambiguous odometry between an old pose and a new pose)

```cpp
// Create a new Hypo-layer in the Hypo-tree to tract the growing of hypotheses
mh_isam2->createNextHypoLayer(mode_size, fac_dim, is_lp, is_t3); 
    //mode_size: number of modes of measurements/association 
    //fac_dim: dimension of the factor, e.g.: 6 for Pose3 
    //is_lp: false (this factor does not create a new loop in the factor graph) 
    //is_t3: does it work with an additional mode that disables all other modes like a Type #3 MMF (usually false in this case)?

// Add a new MHV
mh_init_values.mhInsert( MHGenericValue<T>(std::vector<T> predict_value_arr) );

// Add a new MMF
mh_graph->add( MHNF<T>(Keys, std::vector<T> measured_arr), is_t3 ); 
    //Same is_t3 in createNextHypoLayer()

// Link the new MMF with the new Hypo-layer
mh_isam2->assocLatestHypoLayerWith(MHNF);
```

Case 3. SMF && creating a new loop in the factor graph (e.g.: observation of an existing landmark, loop closure)

```cpp
// Update the overall dimension of the factor graph in the Hypo-tree
mh_isam2->accumulateCommonDim(fac_dim);

// Add a new SMF (no new MHV added in this case)
mh_graph->add( MHNF<T>(Keys, std::vector<T> measured_arr) ); 
    //measured_arr.size() should be 1
```

Case 4. MMF && creating a new loop in the factor graph (e.g.: ambiguous association of an observation to existing landmarks, ambiguous loop closure)

```cpp
// Create a new Hypo-layer in the Hypo-tree to tract the growing of hypotheses
mh_isam2->createNextHypoLayer(mode_size, fac_dim, is_lp, is_t3); 
    //is_lp: true (this factor will create a new loop in the factor graph) 
    //mode_size, fac_dim, is_t3: see case 2.

// Add a new MMF (no new MHV added in this case)
mh_graph->add( MHNF<T>(Keys, std::vector<T> measured_arr), is_t3 ); 
    //Same is_t3 in createNextHypoLayer()

// Link the new MMF with the new Hypo-layer
mh_isam2->assocLatestHypoLayerWith(MHNF);

// Expand the hypotheses of the affected MHV
mh_isam2->setVariableToExpand(Key, last_layer); 
    //Key: the latest MHV in the MMF (should be the latest one in the entire factor graph as well. See notice below.)
```

Notice: MH-iSAM2 is designed as an incremental optimizer. As a result, it works best if each newly added MMF always links to the latest MHV. It may be possible to add MMFs that only link among old MHVs. However, a user has to manually expand the hypotheses of the affected MHVs using setVariableToExpand().


Update MH-iSAM2 / Solve MHVs
----------------------------

The core algorithm of MH-iSAM2 will be conducted in the two following main functions:

```cpp
// One iteration of update in the MHBT based on multi-hypothesis upward message passing
mh_isam2->update( *mh_graph, mh_init_values ); 
    //Prune Hypo-tree and MHVs at the end of each update
    //To run multiple iterations, simply call mh_isam2->update() multiple times

// Reset subgraph and initial values
mh_graph->resize(0);
mh_init_values.clear();

// Estimate the MHVs through multi-hypothesis backsubstitution
mh_results = mh_isam2->mhCalculateBestEstimate();
```

In theory both update() and mhCalculateBestEstimate() can be done at anytime. However, it is highly suggested to update MH-iSAM2 right after adding each MMF into the factor graph.


Access Results
--------------

Access all remaining possible estimates of one MHV:

```cpp
std::vector<T> out_arr = mh_results.mhAt<T>(Key);
```

Access the corresponding solution of a specific hypothesis of one MHV:

```cpp
T out_value = mh_results.mhAtHypo<T>(Key, hypo_ptr); //hypo_ptr: any hypothesis in a later Hypo-layer of the corresponding Hypo-layer of the MHV. 
```

One way to get the above hypo_ptr is to iterate through MHISAM2::HypoList& latest_hypo_list = mh_isam2->getLastHypoLayer()->getNodeList(). A naive function that gets the hypo_ptr of the "best hypothesis" is also implemented:

```cpp
HypoNode* best_hypo_ptr = mh_isam2->getBestHypo(); 
    //Naive definition of best: The remaining hypotheses that has the smallest system error
    //There could be better criteria such as choosing the hypothesis with the highest chi-square confidence.
```

