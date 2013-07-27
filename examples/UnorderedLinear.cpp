
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/linear/GaussianBayesTreeUnordered.h>
#include <gtsam/linear/GaussianJunctionTreeUnordered.h>
#include <gtsam/linear/GaussianEliminationTreeUnordered.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <string>

#include <boost/timer/timer.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/export.hpp>

using namespace gtsam;
using namespace std;

BOOST_CLASS_EXPORT(Value);
BOOST_CLASS_EXPORT(Pose2);
BOOST_CLASS_EXPORT(NonlinearFactor);
BOOST_CLASS_EXPORT(NoiseModelFactor);
BOOST_CLASS_EXPORT(NoiseModelFactor1<Pose2>);
typedef NoiseModelFactor2<Pose2,Pose2> NMF2;
BOOST_CLASS_EXPORT(NMF2);
BOOST_CLASS_EXPORT(BetweenFactor<Pose2>);
BOOST_CLASS_EXPORT(PriorFactor<Pose2>);
BOOST_CLASS_EXPORT(noiseModel::Base);
BOOST_CLASS_EXPORT(noiseModel::Isotropic);
BOOST_CLASS_EXPORT(noiseModel::Gaussian);
BOOST_CLASS_EXPORT(noiseModel::Diagonal);
BOOST_CLASS_EXPORT(noiseModel::Unit);

double chi2_red(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& config) {
  // Compute degrees of freedom (observations - variables)
  // In ocaml, +1 was added to the observations to account for the prior, but
  // the factor graph already includes a factor for the prior/equality constraint.
  //  double dof = graph.size() - config.size();
  int graph_dim = 0;
  BOOST_FOREACH(const boost::shared_ptr<gtsam::NonlinearFactor>& nlf, graph) {
    graph_dim += (int)nlf->dim();
  }
  double dof = double(graph_dim - config.dim()); // kaess: changed to dim
  return 2. * graph.error(config) / dof; // kaess: added factor 2, graph.error returns half of actual error
}

/* ************************************************************************* */
ISAM2 solveWithOldISAM2(const NonlinearFactorGraph& measurements)
{
  ISAM2 isam2;

  size_t nextMeasurement = 0;
  for(size_t step=1; nextMeasurement < measurements.size() && nextMeasurement < 1000; ++step) {

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

  //try {
    //// Load graph
    //gttic_(Find_datafile);
    //string datasetFile = findExampleDataFile("w10000-odom");
    //std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> data = load2D(datasetFile);
    //gttoc_(Find_datafile);

    //NonlinearFactorGraph measurements = *data.first;
    //Values initial = *data.second;

    //// Solve with old ISAM2 to get initial estimate
    //cout << "Solving with old ISAM2 to obtain initial estimate" << endl;
  //  ISAM2 isam = solveWithOldISAM2(measurements);
  //  {
  //    std::ofstream writerStream("incremental_init1000", ios::binary);
  //    boost::archive::binary_oarchive writer(writerStream);
  //    writer << isam.calculateEstimate();
  //    writerStream.close();
  //  }
  //  {
  //    std::ofstream writerStream("incremental_graph1000", ios::binary);
  //    boost::archive::binary_oarchive writer(writerStream);
  //    writer << isam.getFactorsUnsafe();
  //    writerStream.close();
  //  }
  //} catch(std::exception& e) {
  //  cout << e.what() << endl;
  //}

  Values isamsoln;
  NonlinearFactorGraph nlfg;

  {
    std::ifstream readerStream("incremental_init", ios::binary);
    boost::archive::binary_iarchive reader(readerStream);
    reader >> isamsoln;
  }
  {
    std::ifstream readerStream("incremental_graph", ios::binary);
    boost::archive::binary_iarchive reader(readerStream);
    reader >> nlfg;
  }

  // Get linear graph
  cout << "Converting to unordered linear graph" << endl;
  Ordering ordering = *isamsoln.orderingArbitrary();
  Ordering orderingCOLAMD = *nlfg.orderingCOLAMD(isamsoln);
  GaussianFactorGraph gfg = *nlfg.linearize(isamsoln, ordering);
  GaussianFactorGraphUnordered gfgu = convertToUnordered(gfg, ordering);

  //OrderingUnordered orderingUnordered;
  //for(Index j = 0; j < ordering.size(); ++j)
  //  orderingUnordered.push_back(ordering.key(j));

  // Solve linear graph
  cout << "Optimizing unordered graph" << endl;
  VectorValuesUnordered unorderedSolnFinal;
  {
    gttic_(Solve_unordered);
    VectorValuesUnordered unorderedSoln;
    for(size_t i = 0; i < 1; ++i) {
      gttic_(VariableIndex);
      VariableIndexUnordered vi(gfgu);
      gttoc_(VariableIndex);
      gttic_(COLAMD);
      OrderingUnordered orderingUnordered = OrderingUnordered::COLAMD(vi);
      gttoc_(COLAMD);
      gttic_(eliminate);
      GaussianBayesTreeUnordered::shared_ptr bt = gfgu.eliminateMultifrontal(orderingUnordered);
      gttoc_(eliminate);
      gttic_(optimize);
      unorderedSoln = bt->optimize();
      gttoc_(optimize);
    }
    gttoc_(Solve_unordered);
    unorderedSolnFinal = unorderedSoln;
  }

  // Solve linear graph with old code
  cout << "Optimizing using old ordered code" << endl;
  VectorValues orderedSolnFinal;
  {
    Ordering orderingToUse = ordering;
    GaussianFactorGraph::shared_ptr orderedGraph = nlfg.linearize(isamsoln, *nlfg.orderingCOLAMD(isamsoln));
    gttic_(Solve_ordered);
    VectorValues orderedSoln;
    for(size_t i = 0; i < 1; ++i) {
      gttic_(VariableIndex);
      boost::shared_ptr<VariableIndex> vi = boost::make_shared<VariableIndex>(gfg);
      gttoc_(VariableIndex);
      gttic_(COLAMD);
      boost::shared_ptr<Permutation> permutation = inference::PermutationCOLAMD(*vi);
      orderingToUse.permuteInPlace(*permutation);
      gttoc_(COLAMD);
      gttic_(eliminate);
      boost::shared_ptr<GaussianBayesTree> bt = GaussianMultifrontalSolver(*orderedGraph, true).eliminate();
      gttoc_(eliminate);
      gttic_(optimize);
      orderedSoln = optimize(*bt);
      gttoc_(optimize);
    }
    gttoc_(Solve_ordered);
    orderedSolnFinal = orderedSoln;
  }

  // Compare results
  compareSolutions(orderedSolnFinal, orderingCOLAMD, unorderedSolnFinal);

  //GaussianEliminationTreeUnordered(gfgu, orderingUnordered).print("ETree: ");
  //GaussianJunctionTreeUnordered(GaussianEliminationTreeUnordered(gfgu, OrderingUnordered::COLAMD(gfgu))).print("JTree: ");
  //gfgu.eliminateMultifrontal(orderingUnordered)->print("BayesTree: ");

  tictoc_print_();

  return 0;
}