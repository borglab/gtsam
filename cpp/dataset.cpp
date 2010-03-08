/*
 * dataset.cpp
 *
 *   Created on: Jan 22, 2010
 *       Author: nikai
 *  Description: utility functions for loading datasets
 */


#include <fstream>
#include <sstream>
#include <cstdlib>
#include "graph-inl.h"

#include "dataset.h"

using namespace std;
using namespace gtsam;

#define LINESIZE 81920

typedef boost::shared_ptr<Pose2Graph> sharedPose2Graph;
typedef boost::shared_ptr<Pose2Config> sharedPose2Config;

namespace gtsam {

/* ************************************************************************* */
pair<string, boost::optional<SharedDiagonal> > dataset(const string& dataset,  const string& user_path) {
	string path = user_path, set = dataset;
	boost::optional<SharedDiagonal> null_model;
	boost::optional<SharedDiagonal> identity(noiseModel::Unit::Create(3));
	boost::optional<SharedDiagonal> small(noiseModel::Diagonal::Variances(
				gtsam::Vector_(3, 0.0001, 0.0001, 0.0003)));

	if (path.empty()) path = string(getenv("HOME")) + "/";
	if (set.empty()) set = string(getenv("DATASET"));

	/*if (set == "intel") return make_pair(path + "data/iSAM/Laser/intel.graph", null_model);
	if (set == "intel-gfs") return make_pair(path + "data/iSAM/Laser/intel.gfs.graph", null_model);
	if (set == "Killian-gfs") return make_pair(path + "data/iSAM/Laser/Killian.gfs.graph", null_model);
	if (set == "Killian") return make_pair(path + "data/iSAM/Laser/Killian.graph", small);
	if (set == "Killian-noised") return make_pair(path + "data/iSAM/Laser/Killian-noised.graph", null_model);
	if (set == "3") return make_pair(path + "borg/toro/data/2D/w3-odom.graph", identity);
	if (set == "100") return make_pair(path + "borg/toro/data/2D/w100-odom.graph", identity);
	if (set == "10K") return make_pair(path + "borg/toro/data/2D/w10000-odom.graph", identity);
	if (set == "olson") return make_pair(path + "data/iSAM/ISAM2/olson06icra.txt", null_model);
	if (set == "victoria") return make_pair(path + "data/iSAM/ISAM2/victoria_park.txt", null_model);
	if (set == "beijing") return make_pair(path + "data/BeijingData/beijingData_trips.log", null_model);*/

	if (set == "intel") return make_pair(path + "borg/CitySLAM/data/Intel/intel.graph", null_model);
	if (set == "intel-gfs") return make_pair(path + "borg/CitySLAM/data/Intel/intel.gfs.graph", null_model);
	if (set == "Killian-gfs") return make_pair(path + "borg/CitySLAM/data/Killian/Killian.gfs.graph", null_model);
	if (set == "Killian") return make_pair(path + "borg/CitySLAM/data/Killian/Killian.graph", small);
	if (set == "Killian-noised") return make_pair(path + "borg/CitySLAM/data/Killian/Killian-noised.graph", null_model);
	if (set == "3") return make_pair(path + "borg/CitySLAM/data/TORO/w3-odom.graph", identity);
	if (set == "100") return make_pair(path + "borg/CitySLAM/data/TORO/w100-odom.graph", identity);
	if (set == "10K") return make_pair(path + "borg/CitySLAM/data/TORO/w10000-odom.graph", identity);
	if (set == "olson") return make_pair(path + "borg/CitySLAM/data/Olson/olson06icra.graph", null_model);
	if (set == "victoria") return make_pair(path + "borg/CitySLAM/data/VictoriaPark/victoria_park.praph", null_model);
	if (set == "beijing") return make_pair(path + "borg/CitySLAM/data/Beijing/beijingData_trips.graph", null_model);

	// trees
	if (set == "intel_tree") return make_pair(path + "borg/CitySLAM/data/Intel/intel.tree", null_model);
	if (set == "intel-gfs_tree") return make_pair(path + "borg/CitySLAM/data/Intel/intel.gfs.tree", null_model);
	if (set == "3_tree") return make_pair(path + "borg/CitySLAM/data/TORO/w3-odom.tree", identity);
	if (set == "100_tree") return make_pair(path + "borg/CitySLAM/data/TORO/w100-odom.tree", identity);
	if (set == "10K_tree") return make_pair(path + "borg/CitySLAM/data/TORO/w10000-odom.tree", identity);

	//constraints
	if (set == "intel_cnstr") return make_pair(path + "borg/CitySLAM/data/Intel/intel.cnstr", null_model);
	if (set == "intel-gfs_cnstr") return make_pair(path + "borg/CitySLAM/data/Intel/intel.gfs.cnstr", null_model);
	if (set == "3_cnstr") return make_pair(path + "borg/CitySLAM/data/TORO/w3-odom.cnstr", identity);
	if (set == "100_cnstr") return make_pair(path + "borg/CitySLAM/data/TORO/w100-odom.cnstr", identity);
	if (set == "10K_cnstr") return make_pair(path + "borg/CitySLAM/data/TORO/w10000-odom.cnstr", identity);
	return make_pair("unknown", null_model);
}

/* ************************************************************************* */

pair<sharedPose2Graph, sharedPose2Config> load2D(
		pair<string, boost::optional<SharedDiagonal> > dataset,
		int maxID, bool addNoise, bool smart) {
	return load2D(dataset.first, dataset.second, maxID, addNoise, smart);
}

pair<sharedPose2Graph, sharedPose2Config> load2D(const string& filename,
		boost::optional<SharedDiagonal> model, int maxID, bool addNoise, bool smart) {
	cout << "Will try to read " << filename << endl;
	ifstream is(filename.c_str());
	if (!is) {
		cout << "load2D: can not find the file!";
		exit(-1);
	}

	sharedPose2Config poses(new Pose2Config);
	sharedPose2Graph graph(new Pose2Graph);

	string tag;

	// load the poses
	bool firstPose;
	while (is) {
		is >> tag;

		if ((tag == "VERTEX2") || (tag == "VERTEX")) {
			int id;
			double x, y, yaw;
			is >> id >> x >> y >> yaw;
			// optional filter
			if (maxID && id >= maxID) continue;
			poses->insert(id, Pose2(x, y, yaw));
		}
		is.ignore(LINESIZE, '\n');
	}
	is.clear(); /* clears the end-of-file and error flags */
	is.seekg(0, ios::beg);

	// load the factors
	while (is) {
		is >> tag;

		if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "ODOMETRY")) {
			int id1, id2;
			double x, y, yaw;
			is >> id1 >> id2 >> x >> y >> yaw;
			Matrix m = eye(3);
			is >> m(0, 0) >> m(0, 1) >> m(1, 1) >> m(2, 2) >> m(0, 2) >> m(1, 2);
			m(2, 0) = m(0, 2);
			m(2, 1) = m(1, 2);
			m(1, 0) = m(0, 1);

			// optional filter
			if (maxID && (id1 >= maxID || id2 >= maxID)) continue;

			Pose2 l1Xl2(x, y, yaw);

			// SharedGaussian noise = noiseModel::Gaussian::Covariance(m, smart);
			if (!model) {
				Vector variances = Vector_(3,m(0,0),m(1,1),m(2,2));
				model = noiseModel::Diagonal::Variances(variances, smart);
			}

			if (addNoise)
				l1Xl2 = expmap(l1Xl2,(*model)->sample());

			// Insert vertices if pure odometry file
			if (!poses->exists(id1)) poses->insert(id1, Pose2());
			if (!poses->exists(id2)) poses->insert(id2, poses->at(id1) * l1Xl2);

			Pose2Graph::sharedFactor factor(new Pose2Factor(id1, id2, l1Xl2, *model));
			graph->push_back(factor);
		}
		is.ignore(LINESIZE, '\n');
	}

	cout << "load2D read a graph file with " << poses->size() << " vertices and "
			<< graph->nrFactors() << " factors" << endl;

	return make_pair(graph, poses);
}

/* ************************************************************************* */
void save2D(const Pose2Graph& graph, const Pose2Config& config, const SharedDiagonal model,
		const string& filename) {
	typedef Pose2Config::Key Key;

	fstream stream(filename.c_str(), fstream::out);

	// save poses
	Pose2Config::Key key;
	Pose2 pose;
	BOOST_FOREACH(boost::tie(key, pose), config)
		stream << "VERTEX2 " << key.index() << " " <<  pose.x() << " " << pose.y() << " " << pose.theta() << endl;

	// save edges
	Matrix R = model->R();
	Matrix RR = prod(trans(R),R);
	BOOST_FOREACH(boost::shared_ptr<NonlinearFactor<Pose2Config> > factor_, graph) {
		boost::shared_ptr<Pose2Factor> factor = boost::dynamic_pointer_cast<Pose2Factor>(factor_);
		if (!factor) continue;

		pose = inverse(factor->measured());
		stream << "EDGE2 " << factor->key2().index() << " " << factor->key1().index()
				<< " " << pose.x() << " " << pose.y() << " " << pose.theta()
				<< " " << RR(0, 0) << " " << RR(0, 1) << " " << RR(1, 1) << " " << RR(2, 2)
				<< " " << RR(0, 2) << " " << RR(1, 2) << endl;
	}

	stream.close();
}

/* ************************************************************************* */
bool load3D(const string& filename) {
	ifstream is(filename.c_str());
	if (!is) return false;

	while (is) {
		char buf[LINESIZE];
		is.getline(buf, LINESIZE);
		istringstream ls(buf);
		string tag;
		ls >> tag;

		if (tag == "VERTEX3") {
			int id;
			double x, y, z, roll, pitch, yaw;
			ls >> id >> x >> y >> z >> roll >> pitch >> yaw;
		}
	}
	is.clear(); /* clears the end-of-file and error flags */
	is.seekg(0, ios::beg);

	bool edgesOk = true;
	while (is) {
		char buf[LINESIZE];
		is.getline(buf, LINESIZE);
		istringstream ls(buf);
		string tag;
		ls >> tag;

		if (tag == "EDGE3") {
			int id1, id2;
			double x, y, z, roll, pitch, yaw;
			ls >> id1 >> id2 >> x >> y >> z >> roll >> pitch >> yaw;
			Matrix m = eye(6);
			for (int i = 0; i < 6; i++)
				for (int j = i; j < 6; j++)
					ls >> m(i, j);
		}
	}
	return true;
}
}
