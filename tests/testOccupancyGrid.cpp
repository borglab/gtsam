/**
 *      @file testOccupancyGrid.cpp
 *      @date May 14, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */


#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <stdlib.h>
#include <math.h>

using namespace std;
using namespace gtsam;

/**
 * Laser Factor
 * @brief factor that encodes a laser measurements likelihood.
 */

class LaserFactor : public DiscreteFactor{
private:
	//FIX ME
	//m_cells changed to vector<Index>
	DiscreteKeys 	m_cells;	///cells in which laser passes through

public:

	///constructor
	LaserFactor(const DiscreteKeys &cells) : m_cells(cells) {}

	/**
	 * Find value for given assignment of values to variables
	 * return 1000 if any of the non-last cell is occupied and 1 otherwise
	 * Values contains all occupancy values (0 or 1)
	 */
	virtual double operator()(const Values &vals) const{

		// loops through all but the last cell and checks that they are all 0.  Otherwise return 1000.
		for(Index i = 0; i < m_cells.size() - 1; i++){
			if(vals.at(m_cells[i].first) == 1)
				return 1000;
		}

		// check if the last cell hit by the laser is 1.  return 1000 otherwise.
		if(vals.at(m_cells[m_cells.size() - 1].first) == 0)
			return 1000;

		return 1;

	}

	/// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
	virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const{
		throw runtime_error("operator * not implemented");
	}

	virtual operator DecisionTreeFactor() const{
		throw runtime_error("operator DecisionTreeFactor not implemented");
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
	size_t		m_width;			//number of cells wide the grid is
	size_t		m_height;			//number of cells tall the grid is
	double		m_res;				//the resolution at which the grid is created

	DiscreteKeys 	m_cells;	//list of keys of all cells in the grid


public:

	class Occupancy : public Values {
	private:

	public:

	};

	typedef std::vector<double> Marginals;
	///constructor
	///Creates a 2d grid of cells with the origin in the center of the grid
	OccupancyGrid(double width, double height, double resolution){
		m_width 	= 	width/resolution;
		m_height 	= 	height/resolution;
		m_res		=	resolution;

		for(size_t i = 0; i < cellCount(); i++)
			m_cells.push_back(DiscreteKey(i,2));
	}

	Occupancy emptyOccupancy(){
		Occupancy		occupancy;		//mapping from Index to value (0 or 1)
		for(size_t i = 0; i < cellCount(); i++)
			occupancy.insert(pair<Index, size_t>((Index)i,0));

		return occupancy;
	}

	///add a prior
	void addPrior(Index cell, double prior){
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
		//ray trace from pose to range to find all cells the laser passes through
		double x = pose.x();		//start position of the laser
		double y = pose.y();
		double step = m_res/8.0;	//amount to step in each iteration of laser traversal
		DiscreteKey 	key;
		DiscreteKeys 	cells;		//list of keys of cells hit by the laser

		//traverse laser
		for(double i = 0; i < range; i += step){
			//get point on laser
			x = pose.x() + i*cos(pose.theta());
			y = pose.y() + i*sin(pose.theta());

			//get the key of the cell that holds point (x,y)
			key = keyLookup(x,y);

			//add cell to list of cells if it is new
			if(i == 0 || key != cells[cells.size()-1])
				cells.push_back(key);
		}

		for(Index i = 0; i < cells.size(); i++)
			printf("%d,", (int)cells[i].first);

		//add a factor that connects all those cells
		push_back(boost::make_shared<LaserFactor>(cells));

	}

	/// returns the number of cells in the grid
	size_t cellCount() const {
		return m_width*m_height;
	}

	/// returns the key of the cell in which point (x,y) lies.
	DiscreteKey keyLookup(double x, double y) const {
		//move (x,y) to the nearest resolution
		x *= (1.0/m_res);
		y *= (1.0/m_res);

		//round to nearest integer
		x = (double)((int)x);
		y = (double)((int)y);


		//determine index
		x += m_width/2;
		y = m_height/2 - y;

		//bounds checking
		size_t index = y*m_width + x;
		index = index >= m_width*m_height ? -1 : index;

		return m_cells[index];
	}



	/// access a cell in the grid via its row and column
	/*size_t &cell(int row, int col){
		Index index = (Index)(row*m_width + col);
		return m_vals[index];
	}
	const size_t cell(int row, int col) const{
		Index index = (Index)(row*m_width + col);
		return m_vals.at(index);
	}*/

	/// prints an ASCII grid to the console
//	void print() const {
//		Index index;
//		printf("\n");
//		for(size_t i = 0; i < m_height; i++){
//			for(size_t j = 0; j < m_width; j++){
//				printf("%ld ", m_vals.at(index));
//				index++;
//			}
//			printf("\n");
//		}
//	}

	//FIX ME
	//better name
	double laserFactorValue(int index, const Occupancy &occupancy) const{
		return (*factors_[index + 1])(occupancy);
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

		boost::random::mt19937 rng;
		boost::random::uniform_int_distribution<Index> six(0,size-1);

		// run Metropolis for the requested number of operations
		// compute initial probability of occupancy grid, P(x_t)
		double Px = (*this)(occupancy);
		for(size_t it; it < iterations; it++){
			//choose a random cell
			Index x = six(rng);

			//flip the state of a random cell, x
				 occupancy[x] = 1 - occupancy[x];

				//compute probability of new occupancy grid, P(x')
					// sum over all LaserFactor::operator()
					double Px_prime = (*this)(occupancy);

			//calculate acceptance ratio, a
				double a = Px_prime/Px;


			//if a >= 1 otherwise accept with probability a
			//if we accept the new state P(x_t) = P(x')
				if(a >= 1){
					Px = Px_prime;
				}else{
					 occupancy[x] = 1 - occupancy[x];
				}
		}

		return marginals;
	}

};

/* ************************************************************************* */
TEST_UNSAFE( OccupancyGrid, Test1) {
	//Build a small grid and test optimization

	//Build small grid
	double width 		=	3; 		//meters
	double height 		= 	2; 		//meters
	double resolution 	= 	0.5; 	//meters
	OccupancyGrid occupancyGrid(width, height, resolution); //default center to middle

	//Add measurements
	Pose2 pose(0,0,0);
	double range = 1.0;

	occupancyGrid.addPrior(0, 0.7);
	EXPECT_LONGS_EQUAL(1, occupancyGrid.size());

	occupancyGrid.addLaser(pose, range);
	EXPECT_LONGS_EQUAL(2, occupancyGrid.size());


	OccupancyGrid::Occupancy occupancy = occupancyGrid.emptyOccupancy();
	EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));


	occupancy[16] = 1;
	EXPECT_LONGS_EQUAL(1, occupancyGrid.laserFactorValue(0,occupancy));

	occupancy[15] = 1;
	EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));

	occupancy[16] = 0;
	EXPECT_LONGS_EQUAL(1000, occupancyGrid.laserFactorValue(0,occupancy));


	//run MCMC
	OccupancyGrid::Marginals occupancyMarginals = occupancyGrid.runMetropolis(5);
	EXPECT_LONGS_EQUAL( (width*height)/pow(resolution,2), occupancyMarginals.size());
	//select a cell at a random to flip

}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

