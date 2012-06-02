/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file dataset.cpp
 * @date Jan 22, 2010
 * @author nikai
 * @brief utility functions for loading datasets
 */


#include <fstream>
#include <sstream>
#include <cstdlib>

#include <gtsam/slam/dataset.h>

using namespace std;
using namespace gtsam;

#define LINESIZE 81920

typedef boost::shared_ptr<pose2SLAM::Graph> sharedPose2Graph;
typedef pose2SLAM::Odometry Pose2Factor; ///< Typedef for Constraint class for backwards compatibility

namespace gtsam {

/* ************************************************************************* */
pair<string, boost::optional<SharedDiagonal> > dataset(const string& dataset,  const string& user_path) {
	string path = user_path, set = dataset;
	boost::optional<SharedDiagonal> null_model;
	boost::optional<SharedDiagonal> identity(noiseModel::Unit::Create(3));
	boost::optional<SharedDiagonal> small(noiseModel::Diagonal::Variances(
				gtsam::Vector_(3, 0.0001, 0.0001, 0.0003)));

	if (path.empty()) path = string(getenv("HOME")) + "/data";
	if (set.empty()) set = string(getenv("DATASET"));

	if (set == "intel") return make_pair(path + "/Intel/intel.graph", null_model);
	if (set == "intel-gfs") return make_pair(path + "/Intel/intel.gfs.graph", null_model);
	if (set == "Killian-gfs") return make_pair(path + "/Killian/Killian.gfs.graph", null_model);
	if (set == "Killian") return make_pair(path + "/Killian/Killian.graph", small);
	if (set == "Killian-noised") return make_pair(path + "/Killian/Killian-noised.graph", null_model);
	if (set == "3") return make_pair(path + "/TORO/w3-odom.graph", identity);
	if (set == "100") return make_pair(path + "/TORO/w100-odom.graph", identity);
	if (set == "10K") return make_pair(path + "/TORO/w10000-odom.graph", identity);
	if (set == "10K2") return make_pair(path + "/hogman/data/2D/w10000.graph",
			noiseModel::Diagonal::Variances(gtsam::Vector_(3, 0.1, 0.1, 0.05)));
	if (set == "Eiffel100") return make_pair(path + "/TORO/w100-Eiffel.graph", identity);
	if (set == "Eiffel10K") return make_pair(path + "/TORO/w10000-Eiffel.graph", identity);
	if (set == "olson") return make_pair(path + "/Olson/olson06icra.graph", null_model);
	if (set == "victoria") return make_pair(path + "/VictoriaPark/victoria_park.graph", null_model);
	if (set == "beijing") return make_pair(path + "/Beijing/beijingData_trips.graph", null_model);

	return make_pair("unknown", null_model);
}

/* ************************************************************************* */

pair<sharedPose2Graph, Values::shared_ptr> load2D(
		pair<string, boost::optional<SharedDiagonal> > dataset,
		int maxID, bool addNoise, bool smart) {
	return load2D(dataset.first, dataset.second, maxID, addNoise, smart);
}

pair<sharedPose2Graph, Values::shared_ptr> load2D(const string& filename,
		boost::optional<SharedDiagonal> model, int maxID, bool addNoise, bool smart) {
	cout << "Will try to read " << filename << endl;
	ifstream is(filename.c_str());
	if (!is) {
		cout << "load2D: can not find the file!";
		exit(-1);
	}

	Values::shared_ptr poses(new Values);
	sharedPose2Graph graph(new pose2SLAM::Graph);

	string tag;

	// load the poses
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

			// SharedNoiseModel noise = noiseModel::Gaussian::Covariance(m, smart);
			if (!model) {
				Vector variances = Vector_(3,m(0,0),m(1,1),m(2,2));
				model = noiseModel::Diagonal::Variances(variances, smart);
			}

			if (addNoise)
				l1Xl2 = l1Xl2.retract((*model)->sample());

			// Insert vertices if pure odometry file
			if (!poses->exists(id1)) poses->insert(id1, Pose2());
			if (!poses->exists(id2)) poses->insert(id2, poses->at<Pose2>(id1) * l1Xl2);

			pose2SLAM::Graph::sharedFactor factor(new Pose2Factor(id1, id2, l1Xl2, *model));
			graph->push_back(factor);
		}
		is.ignore(LINESIZE, '\n');
	}

	cout << "load2D read a graph file with " << poses->size() << " vertices and "
			<< graph->nrFactors() << " factors" << endl;

	return make_pair(graph, poses);
}

/* ************************************************************************* */
void save2D(const pose2SLAM::Graph& graph, const Values& config, const SharedDiagonal model,
		const string& filename) {

	fstream stream(filename.c_str(), fstream::out);

	// save poses
	BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, config) {
	  const Pose2& pose = dynamic_cast<const Pose2&>(key_value.value);
		stream << "VERTEX2 " << key_value.key << " " <<  pose.x() << " " << pose.y() << " " << pose.theta() << endl;
	}

	// save edges
	Matrix R = model->R();
	Matrix RR = trans(R)*R;//prod(trans(R),R);
	BOOST_FOREACH(boost::shared_ptr<NonlinearFactor> factor_, graph) {
		boost::shared_ptr<Pose2Factor> factor = boost::dynamic_pointer_cast<Pose2Factor>(factor_);
		if (!factor) continue;

		Pose2 pose = factor->measured().inverse();
		stream << "EDGE2 " << factor->key2() << " " << factor->key1()
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
