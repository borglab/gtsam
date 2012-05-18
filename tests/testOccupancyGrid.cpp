/**
 *      @file testOccupancyGrid.cpp
 *      @date May 14, 2012
 *      @author Brian Peasley
 *      @author Frank Dellaert
 */


#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <CppUnitLite/TestHarness.h>
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
	DiscreteKeys 	m_cells;
public:

	///constructor
	LaserFactor(const DiscreteKeys &cells) {
		m_cells.resize(cells.size());
		for(unsigned int i = 0; i < cells.size(); i++)
			m_cells[i] = cells[i];
	}

	/// Find value for given assignment of values to variables
	/// return 1000 if any of the non-last cell is occupied and 1 otherwise
	/// Values contains all occupancy values (0 or 1)
	virtual double operator()(const Values &vals) const{

		for(unsigned int i = 0; i < m_cells.size() - 1; i++){
			if(vals.at(m_cells[i].first) == 1)
				return 1000;
		}

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
	int		m_width;			//number of cells wide the grid is
	int		m_height;			//number of cells tall the grid is
	double	m_res;				//the resolution at which the grid is created

	DiscreteKeys 	m_cells;	//list of keys of all cells in the grid
	Values			m_vals;		//mapping from Index to value (0 or 1)

public:

	///constructor
	///Creates a 2d grid of cells with the origin in the center of the grid
	OccupancyGrid(double width, double height, double resolution){
		m_width 	= 	width/resolution;
		m_height 	= 	height/resolution;
		m_res		=	resolution;

		for(int i = 0; i < cellCount(); i++){
			m_cells.push_back(DiscreteKey(i,2));
			m_vals.insert(pair<Index, size_t>((Index)i,0));
		}
		m_vals[0];

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

		for(unsigned int i = 0; i < cells.size(); i++)
			printf("%d,", (int)cells[i].first);

		//add a factor that connects all those cells
		push_back(boost::make_shared<LaserFactor>(cells));

	}

	/// returns the number of cells in the grid
	int cellCount() const {
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
		int index = y*m_width + x;
		index = index >= m_width*m_height ? -1 : index;

		return m_cells[index];
	}

	/// access a cell in the grid via its index
		size_t &operator[](Index index){
			return m_vals[index];
		}
		const size_t operator[](Index index) const{
			return m_vals.at(index);
		}

	/// access a cell in the grid via its row and column
	size_t &operator()(int row, int col){
		Index index = (Index)(row*m_width + col);
		return m_vals[index];
	}
	const size_t operator()(int row, int col) const{
		Index index = (Index)(row*m_width + col);
		return m_vals.at(index);
	}

	/// prints an ASCII grid to the console
	void print() const {
		Index index;
		printf("\n");
		for(int i = 0; i < m_height; i++){
			for(int j = 0; j < m_width; j++){
				printf("%ld ", m_vals.at(index));
				index++;
			}
			printf("\n");
		}
	}
	double operator()(int index) const{
		return (*factors_[index + 1])(m_vals);
	}

	void assignments()const {
		m_vals.print();
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
	EXPECT_LONGS_EQUAL(1000, occupancyGrid(0));

	occupancyGrid[16] = 1;
	EXPECT_LONGS_EQUAL(1, occupancyGrid(0));

	occupancyGrid[15] = 1;
	EXPECT_LONGS_EQUAL(1000, occupancyGrid(0));

	occupancyGrid[16] = 0;
	EXPECT_LONGS_EQUAL(1000, occupancyGrid(0));


	//run MCMC
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

