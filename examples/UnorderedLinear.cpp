
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <string>

#include <boost/timer/timer.hpp>

using namespace gtsam;
using namespace std;

double chi2_red(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& config) {
  // Compute degrees of freedom (observations - variables)
  // In ocaml, +1 was added to the observations to account for the prior, but
  // the factor graph already includes a factor for the prior/equality constraint.
  //  double dof = graph.size() - config.size();
  int graph_dim = 0;
  BOOST_FOREACH(const boost::shared_ptr<gtsam::NonlinearFactor>& nlf, graph) {
    graph_dim += nlf->dim();
  }
  double dof = graph_dim - config.dim(); // kaess: changed to dim
  return 2. * graph.error(config) / dof; // kaess: added factor 2, graph.error returns half of actual error
}

/* ************************************************************************* */
ISAM2 solveWithOldISAM2(const NonlinearFactorGraph& measurements)
{
  ISAM2 isam2;

  size_t nextMeasurement = 0;
  for(size_t step=1; nextMeasurement < measurements.size(); ++step) {

    Values newVariables;
    NonlinearFactorGraph newFactors;

    // Collect measurements and new variables for the current step
    gttic_(Collect_measurements);
    if(step == 1) {
      //      cout << "Initializing " << 0 << endl;
      newVariables.insert(0, Pose2());
      // Add prior
      newFactors.add(PriorFactor<Pose2>(0, Pose2(), noiseModel::Unit::Create(Pose2::Dim())));
    }
    while(nextMeasurement < measurements.size()) {

      NonlinearFactor::shared_ptr measurementf = measurements[nextMeasurement];

      if(BetweenFactor<Pose2>::shared_ptr measurement =
        boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(measurementf))
      {
        // Stop collecting measurements that are for future steps
        if(measurement->key1() > step || measurement->key2() > step)
          break;

        // Require that one of the nodes is the current one
        if(measurement->key1() != step && measurement->key2() != step)
          throw runtime_error("Problem in data file, out-of-sequence measurements");

        // Add a new factor
        newFactors.push_back(measurement);

        // Initialize the new variable
        if(measurement->key1() == step && measurement->key2() == step-1) {
          if(step == 1)
            newVariables.insert(step, measurement->measured().inverse());
          else {
            Pose2 prevPose = isam2.calculateEstimate<Pose2>(step-1);
            newVariables.insert(step, prevPose * measurement->measured().inverse());
          }
          //        cout << "Initializing " << step << endl;
        } else if(measurement->key2() == step && measurement->key1() == step-1) {
          if(step == 1)
            newVariables.insert(step, measurement->measured());
          else {
            Pose2 prevPose = isam2.calculateEstimate<Pose2>(step-1);
            newVariables.insert(step, prevPose * measurement->measured());
          }
          //        cout << "Initializing " << step << endl;
        }
      } else {
        throw std::runtime_error("Unknown factor type read from data file");
      }
      ++ nextMeasurement;
    }
    gttoc_(Collect_measurements);

    // Update iSAM2
    gttic_(Update_ISAM2);
    isam2.update(newFactors, newVariables);
    gttoc_(Update_ISAM2);

    tictoc_finishedIteration_();

    if(step % 1000 == 0) {
      cout << "Step " << step << endl;
      tictoc_print_();
    }
  }

  return isam2;
}

/* ************************************************************************* */
GaussianFactorGraphUnordered convertToUnordered(const GaussianFactorGraph& gfg, const Ordering& ordering)
{
  GaussianFactorGraphUnordered gfgu;
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, gfg)
  {
    vector<std::pair<Key, Matrix> > terms;
    
    const JacobianFactor& jacobian = dynamic_cast<const JacobianFactor&>(*factor);
    for(GaussianFactor::const_iterator term = jacobian.begin(); term != jacobian.end(); ++term)
    {
      terms.push_back(make_pair(
        ordering.key(*term),
        jacobian.getA(term)));
    }

    gfgu.add(JacobianFactorUnordered(terms, jacobian.getb(), jacobian.get_model()));
  }
  return gfgu;
}

/* ************************************************************************* */
void compareSolutions(const VectorValues& orderedSoln, const Ordering& ordering, const VectorValuesUnordered& unorderedSoln)
{
  if(orderedSoln.size() != unorderedSoln.size())
  {
    cout << "Solution sizes are not the same" << endl;
  }
  else
  {
    double maxErr = 0.0;
    BOOST_FOREACH(const VectorValuesUnordered::KeyValuePair& v, unorderedSoln)
    {
      Vector orderedV = orderedSoln[ordering[v.first]];
      maxErr = std::max(maxErr, (orderedV - v.second).cwiseAbs().maxCoeff());
    }
    cout << "Maximum abs error: " << maxErr << endl;
  }
}

/* ************************************************************************* */
int main(int argc, char *argv[])
{
  // Load graph
  gttic_(Find_datafile);
  string datasetFile = findExampleDataFile("w10000-odom");
  std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> data = load2D(datasetFile);
  gttoc_(Find_datafile);

  NonlinearFactorGraph measurements = *data.first;
  Values initial = *data.second;

  // Solve with old ISAM2 to get initial estimate
  cout << "Solving with old ISAM2 to obtain initial estimate" << endl;
  gttic_(Old_ISAM2);
  ISAM2 isam = solveWithOldISAM2(measurements);
  Values isamsoln = isam.calculateEstimate();
  gttoc_(Old_ISAM2);

  // Get linear graph
  cout << "Converting to unordered linear graph" << endl;
  Ordering ordering = *isam.getFactorsUnsafe().orderingCOLAMD(isamsoln);
  GaussianFactorGraph gfg = *isam.getFactorsUnsafe().linearize(isamsoln, ordering);
  GaussianFactorGraphUnordered gfgu = convertToUnordered(gfg, ordering);

  // Solve linear graph
  cout << "Optimizing unordered graph" << endl;
  {
    boost::timer::cpu_timer timer;
    gttic_(Solve_unordered);
    for(size_t i = 0; i < 20; ++i)
      VectorValuesUnordered unorderedSoln = gfgu.optimize();
    gttoc_(Solve_unordered);
    timer.stop();
    boost::timer::cpu_times times = timer.elapsed();
    std::cout << "Total CPU time: " << double(times.system + times.user) / 1e9
      << ", wall time: " << double(times.wall) / 1e9 << std::endl;
  }

  // Solve linear graph with old code
  cout << "Optimizing using old ordered code" << endl;
  {
    boost::timer::cpu_timer timer;
    gttic_(Solve_ordered);
    for(size_t i = 0; i < 20; ++i)
      VectorValues orderedSoln = *GaussianMultifrontalSolver(gfg, true).optimize();
    gttoc_(Solve_ordered);
    timer.stop();
    boost::timer::cpu_times times = timer.elapsed();
    std::cout << "Total CPU time: " << double(times.system + times.user) / 1e9
      << ", wall time: " << double(times.wall) / 1e9 << std::endl;
  }

  // Compare results
  //compareSolutions(orderedSoln, ordering, unorderedSoln);

  tictoc_print_();

  return 0;
}