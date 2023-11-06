/**
 *      @file testOccupancyGrid.cpp
 *      @date May 14, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#if 0

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>

#include <vector>
#include <cmath.>
#include <random>

#include <stdlib.h>

using namespace std;
using namespace gtsam;


/**
 * Laser Factor
 * @brief factor that encodes a laser measurements likelihood.
 */

class LaserFactor : public DiscreteFactor{
private:
  vector<Index>   m_cells;  ///cells in which laser passes through

public:

  ///constructor
  LaserFactor(const vector<Index> &cells) : m_cells(cells) {}

  /**
   * Find value for given assignment of values to variables
   * return 1000 if any of the non-last cell is occupied and 1 otherwise
   * Values contains all occupancy values (0 or 1)
   */
  virtual double operator()(const Values &vals) const{

    // loops through all but the last cell and checks that they are all 0.  Otherwise return 1000.
    for(Index i = 0; i < m_cells.size() - 1; i++){
      if(vals.at(m_cells[i]) == 1)
        return 1000;
    }

    // check if the last cell hit by the laser is 1.  return 900 otherwise.
    if(vals.at(m_cells[m_cells.size() - 1]) == 0)
      return 900;

    return 1;

  }

  /// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
  virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const{
    throw runtime_error("operator * not implemented");
  }

  virtual DecisionTreeFactor toDecisionTreeFactor() const{
    throw runtime_error("DecisionTreeFactor toDecisionTreeFactor not implemented");
  }
};

/**
 * OccupancyGrid Class
 * An occupancy grid is just a factor graph.
 * Every cell in the occupancy grid is a variable in the factor graph.
 * Measurements will create factors, as well as the prior.
 */
class OccupancyGrid : public DiscreteFactorGraph {
private:
  size_t      width_;    //number of cells wide the grid is
  size_t      height_;  //number of cells tall the grid is
  double      res_;    //the resolution at which the grid is created

  vector<Index>   cells_;      //list of keys of all cells in the grid
  vector<Index>   laser_indices_; //indices of the laser factor in factors_


public:

  size_t width() const {
    return width_;
  }
  size_t height() const {
    return height_;
  }
  // should we just not typedef Values Occupancy; ?
  class Occupancy : public Values {
  private:
  public:
  };


  typedef std::vector<double> Marginals;
  ///constructor
  ///Creates a 2d grid of cells with the origin in the center of the grid
  OccupancyGrid(double width, double height, double resolution){
    width_     =   width/resolution;
    height_   =   height/resolution;
    res_    =  resolution;

    for(Index i = 0; i < cellCount(); i++)
      cells_.push_back(i);
  }

  /// Returns an empty occupancy grid of size width_ x height_
  Occupancy emptyOccupancy(){
    Occupancy    occupancy;    //mapping from Index to value (0 or 1)
    for(size_t i = 0; i < cellCount(); i++)
      occupancy.insert(pair<Index, size_t>((Index)i,0));

    return occupancy;
  }

  ///add a prior
  void addPosePrior(Index cell, double prior){
    size_t numStates = 2;
    DiscreteKey key(cell, numStates);

    //add a factor
    vector<double> table(2);
    table[0] = 1-prior;
    table[1] = prior;
    add(key, table);
  }

  ///add a laser measurement
  void addLaser(const Pose2 &pose, double range){
    //ray trace from pose to range t//a >= 1 accept new stateo find all cells the laser passes through
    double x = pose.x();    //start position of the laser
    double y = pose.y();
    double step = res_/8.0;  //amount to step in each iteration of laser traversal

    Index       key;
    vector<Index>   cells;    //list of keys of cells hit by the laser

    //traverse laser
    for(double i = 0; i < range; i += step){
      //get point on laser
      x = pose.x() + i*cos(pose.theta());
      y = pose.y() + i*sin(pose.theta());

      //printf("%lf %lf\n", x, y);
      //get the key of the cell that holds point (x,y)
      key = keyLookup(x,y);

      //add cell to list of cells if it is new
      if(i == 0 || key != cells[cells.size()-1])
        cells.push_back(key);
    }

//    for(size_t i = 0; i < cells.size(); i++)
//      printf("%ld ", cells[i]);
//    printf("\n");

    //add a factor that connects all those cells
    laser_indices_.push_back(factors_.size());
    push_back(std::make_shared<LaserFactor>(cells));

  }

  /// returns the number of cells in the grid
  size_t cellCount() const {
    return width_*height_;
  }

  /// returns the key of the cell in which point (x,y) lies.
  Index keyLookup(double x, double y) const {
    //move (x,y) to the nearest resolution
    x *= (1.0/res_);
    y *= (1.0/res_);

    //round to nearest integer
    x = (double)((int)x);
    y = (double)((int)y);

    //determine index
    x += width_/2;
    y = height_/2 - y;

    //bounds checking
    size_t index = y*width_ + x;
    index = index >= width_*height_ ? -1 : index;

    return cells_[index];
  }

  /**
   * @brief Computes the value of a laser factor
   * @param index defines which laser is to be used
   * @param occupancy defines the grid which the laser will be evaulated with
   * @ret a double value that is the value of the specified laser factor for the grid
   */
  double laserFactorValue(Index index, const Occupancy &occupancy) const{
    return (*factors_[ laser_indices_[index] ])(occupancy);
  }

  /// returns the sum of the laser factors for the current state of the grid
  double operator()(const Occupancy &occupancy) const {
    double value = 0;

    // loop over all laser factors in the graph
    //printf("%ld\n", (*this).size());

    for(Index i = 0; i < laser_indices_.size(); i++){
      value += laserFactorValue(i, occupancy);
    }

    return value;
  }

  /**
   * @brief Run a metropolis sampler.
   * @param iterations defines the number of iterations to run.
   * @return  vector of marginal probabilities.
   */
  Marginals runMetropolis(size_t iterations){
    Occupancy occupancy = emptyOccupancy();

    size_t size = cellCount();
    Marginals marginals(size);

    // NOTE: using older interface for boost.random due to interface changes after boost 1.46
    std::mt19937 rng;
    std::uniform_int_distribution<> random_cell(0, size - 1);

    // run Metropolis for the requested number of operations
    // compute initial probability of occupancy grid, P(x_t)

    double Px = (*this)(occupancy);

    for(size_t it = 0; it < marginals.size(); it++)
      marginals[it] = 0;

    for(size_t it = 0; it < iterations; it++){
      //choose a random cell
      Index x = random_cell(rng);
      //printf("%ld:",x);
      //flip the state of a random cell, x
         occupancy[x] = 1 - occupancy[x];

      //compute probability of new occupancy grid, P(x')
      //by summing over all LaserFactor::operator()
         double Px_prime = (*this)(occupancy);

      //occupancy.print();
      //calculate acceptance ratio, a
        double a = Px_prime/Px;

      //if a <= 1 otherwise accept with probability a
      //if we accept the new state P(x_t) = P(x')
      //  printf(" %.3lf %.3lf\t", Px, Px_prime);
        if(a <= 1){
          Px = Px_prime;
          //printf("\taccept\n");
        }
        else{
           occupancy[x] = 1 - occupancy[x];
          // printf("\treject\n");
        }

      //increment the number of iterations each cell has been on
        for(size_t i = 0; i < size; i++){
          if(occupancy[i] == 1)
            marginals[i]++;
        }
    }

    //compute the marginals
    for(size_t it = 0; it < size; it++)
      marginals[it] /= iterations;

    return marginals;
  }

};

/* ************************************************************************* */
TEST( OccupancyGrid, Test1) {
  //Build a small grid and test optimization

  //Build small grid
  double width     =  3;     //meters
  double height     =   2;     //meters
  double resolution   =   0.5;   //meters
  OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle

  //Add measurements
  Pose2 pose(0,0,0);
  double range = 1;

  occupancyGrid.addPosePrior(0, 0.7);
  EXPECT_LONGS_EQUAL(1, occupancyGrid.size());

  occupancyGrid.addLaser(pose, range);
  EXPECT_LONGS_EQUAL(2, occupancyGrid.size());

  OccupancyGrid::Occupancy occupancy = occupancyGrid.emptyOccupancy();
  EXPECT_LONGS_EQUAL(900, occupancyGrid.laserFactorValue(0,occupancy));


  occupancy[16] = 1;
  EXPECT_LONGS_EQUAL(1, occupancyGrid.laserFactorValue(0,occupancy));

  occupancy[15] = 1;
  EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));

  occupancy[16] = 0;
  EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));


  //run MCMC
  OccupancyGrid::Marginals occupancyMarginals = occupancyGrid.runMetropolis(50000);
  EXPECT_LONGS_EQUAL( (width*height)/pow(resolution,2), occupancyMarginals.size());



}

#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

