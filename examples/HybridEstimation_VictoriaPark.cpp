#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridNonlinearISAM.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;

using symbol_shorthand::K;
using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

/**
 * A class for a measurement of a Point2 from a Pose2
 * @tparam VALUE the Value type
 * @addtogroup SLAM
 */
class Pose2Point2Factor : public NoiseModelFactor2<Pose2, Point2> {
 private:
  typedef Pose2Point2Factor This;
  typedef NoiseModelFactor2<Pose2, Point2> Base;

  Point2 measured_; /** The measurement */

 public:
  // shorthand for a smart pointer to a factor
  typedef typename std::shared_ptr<Pose2Point2Factor> shared_ptr;

  /** default constructor - only use for serialization */
  Pose2Point2Factor() {}

  /** Constructor */
  Pose2Point2Factor(Key key1, Key key2, const Point2& measured,
                    const SharedNoiseModel& model)
      : Base(model, key1, key2), measured_(measured) {}

  virtual ~Pose2Point2Factor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** implement functions needed for Testable */

  /** print */
  virtual void print(
      const std::string& s,
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "Pose2Point2Factor(" << keyFormatter(this->key1()) << ","
              << keyFormatter(this->key2()) << ")\n";
    std::cout << "measured: " << measured_ << std::endl;
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  virtual bool equals(const NonlinearFactor& expected,
                      double tol = 1e-9) const override {
    /*
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != NULL && Base::equals(*e, tol) &&
    traits<T>::Equals(this->measured_, e->measured_, tol);
    // */
    std::cout << "Pose2Point2Factor::equals() NOT implemented yet" << std::endl;
    return false;
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const Pose2& pose, const Point2& point,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    Point2 predicted_point = pose.transformTo(point, H1, H2);
    return predicted_point - measured_;
  }

  /** return the measured */
  const Point2& measured() const { return measured_; }

  /** number of variables attached to this factor */
  std::size_t size() const { return 2; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_);
  }
};  // \class BetweenFactor

/// traits
template <>
struct traits<Pose2Point2Factor> : public Testable<Pose2Point2Factor> {};

// Testing params
const size_t max_odom_count = 1000;  // 3800;  // 3800

noiseModel::Diagonal::shared_ptr prior_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(3) << 0.0001, 0.0001, 0.0001).finished());

noiseModel::Diagonal::shared_ptr pose_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(3) << 1.0 / 100.0, 1.0 / 500.0, 1.0 / 500.0).finished());

noiseModel::Diagonal::shared_ptr point_noise_model =
    noiseModel::Diagonal::Sigmas(
        (Vector(2) << 1.0 / 1.581139, 1.0 / 1.581139).finished());

/* ************************************************************************* */

int main(int argc, char* argv[]) {
  // Type #2 only
  string dataset_file = findExampleDataFile("T2_victoriaPark_01.txt");
  // Type #1 + Type #2
  //  string dataset_file = findExampleDataFile("T1_T2_victoriaPark_08.txt");
  ifstream in(dataset_file);

  size_t odom_count = 0;
  size_t landmark_count = 0;

  std::list<double> time_list;

  HybridNonlinearISAM isam;

  HybridNonlinearFactorGraph graph;
  Values initValues;

  double x = 0.0;
  double y = 0.0;
  double rad = 0.0;

  std::vector<Pose2> prior_arr;
  prior_arr.push_back(Pose2(x, y, rad));

  Pose2 prior_value(prior_arr[0]);
  initValues.insert(X(0), prior_value);
  graph.emplace_shared<PriorFactor<Pose2>>(X(0), prior_value,
                                           prior_noise_model);

  //*
  size_t key_s = 0;
  size_t key_t = 0;

  clock_t start_time = clock();
  string str;
  while (getline(in, str) && odom_count < max_odom_count) {
    vector<string> parts;
    split(parts, str, is_any_of(" "));

    if (parts[0] == "ODOMETRY") {
      cout << "--------- odom count: " << odom_count << endl;

      key_s = stoi(parts[1]);
      key_t = stoi(parts[3]);
      int m_num = stoi(parts[5]);
      vector<double> x_arr(m_num);
      vector<double> y_arr(m_num);
      vector<double> rad_arr(m_num);
      for (int i = 0; i < m_num; ++i) {
        x_arr[i] = stod(parts[6 + 3 * i]);
        y_arr[i] = stod(parts[7 + 3 * i]);
        rad_arr[i] = stod(parts[8 + 3 * i]);
      }

      if (m_num > 1) {
        std::cout << "hybrid at odom_count: " << odom_count << std::endl;
        exit(0);
        std::vector<NoiseModelFactor::shared_ptr> factors;
        for (int i = 0; i < m_num; ++i) {
          Pose2 measurement(x_arr[i], y_arr[i], rad_arr[i]);
          factors.push_back(std::make_shared<BetweenFactor<Pose2>>(
              X(key_s), X(key_t), measurement, pose_noise_model));
        }
        graph.add(HybridNonlinearFactor(DiscreteKey(M(key_s), m_num), factors));
      } else {
        Pose2 measurement(x_arr[0], y_arr[0], rad_arr[0]);
        graph.emplace_shared<BetweenFactor<Pose2>>(
            X(key_s), X(key_t), measurement, pose_noise_model);
      }

      Pose2 pose_value(x_arr[0], y_arr[0], rad_arr[0]);
      initValues.insert(X(key_t), pose_value);

      odom_count++;
      // cout << odom_count << endl;

    } else {  // LANDMARK
      //*
      key_s = stoi(parts[1]);
      int k_num = stoi(parts[2]);

      if (k_num == 1) {
        key_t = stoi(parts[3]);
        // int m_num = 1;
        double l_x = stod(parts[6]);
        double l_y = stod(parts[7]);

        std::vector<Point2> measured_arr;
        measured_arr.push_back(Point2(l_x, l_y));

        if (key_t >= landmark_count) {
          initValues.insert(L(key_t), measured_arr[0]);
          landmark_count++;
        }

        graph.add(Pose2Point2Factor(X(key_s), L(key_t), measured_arr[0],
                                    point_noise_model));

        // landmark_count++;
      } else {
        size_t key_t1 = stoi(parts[3]);
        size_t key_t2 = stoi(parts[4]);

        double l_x = stod(parts[7]);
        double l_y = stod(parts[8]);

        Point2 measured_point(l_x, l_y);

        // graph.add(HybridNonlinearFactor(
        //     DiscreteKey(K(key_s), 2),
        //     {std::make_shared<Pose2Point2Factor>(
        //          X(key_s), L(key_t1), measured_point, point_noise_model),
        //      std::make_shared<Pose2Point2Factor>(
        //          X(key_s), L(key_t2), measured_point, point_noise_model)}));
        // graph.add(std::make_shared<Pose2Point2Factor>(
        //     X(key_s), L(key_t1), measured_point, point_noise_model));
      }
    }

    isam.update(graph, initValues);
    graph.resize(0);
    initValues.clear();

    if (parts[0] == "ODOMETRY") {
      clock_t cur_time = clock();
      time_list.push_back(cur_time - start_time);
    }

    // if (time_list.size() % 100 == 0) {
    //   string step_file_idx = std::to_string(100000 + time_list.size());

    //   ofstream step_outfile;
    //   string step_file_name =
    //       "mh_T2_step_files/MH_ISAM2_TEST_victoriaPark_S" + step_file_idx;
    //   step_outfile.open(step_file_name + ".txt");

    //   MHISAM2::HypoListIter it =
    //       mh_isam2->getLastHypoLayer()->getNodeList().begin();
    //   for (size_t i = 0; i < (odom_count + 1); ++i) {
    //     Pose2 out_pose = mh_results.mhAtHypo<Pose2>(X(i), (*it));

    //     Vector tt = out_pose.translation().vector();
    //     step_outfile << tt(0) << " " << tt(1) << " ";  // [x1 y1] [x2 y2]
    //     ...
    //   }
    //   step_outfile.close();
    // }

    // //*
    // if (odom_count % 50 == 0 && key_s != key_t - 1) {
    //   std::cout << "odom_count: " << odom_count << std::endl;

    //   // MHISAM2::HypoList& curr_hypo_list =
    //   // mh_isam2->getLastHypoLayer()->getNodeList(); std::cout << "Last
    //   layer
    //   // hypos:  " << curr_hypo_list.size() << std::endl;

    //   std::cout << "acc_time:  " << time_list.back() << std::endl;
    // }
    // */
  }

  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time << endl;
  //*

  Values results = isam.estimate();

  ofstream outfile;
  string file_name = "HybridISAM_victoriaPark.txt";
  outfile.open(file_name);

  for (size_t i = 0; i < odom_count; ++i) {
    Pose2 pose = results.at<Pose2>(X(i));

    // WRONG FORMAT... JUST FOR SIMPLE SHOW
    outfile << pose.x() << " " << pose.y() << " " << pose.theta() << endl;
  }

  outfile.close();
  cout << "output " << file_name << " file." << endl;
  // */
  ofstream lm_outfile;
  string lm_file_name = "HybridISAM_victoriaPark_lm.txt";
  lm_outfile.open(lm_file_name);
  // outfile << "total_time:  " << total_time << endl;
  for (size_t i = 0; i < landmark_count; ++i) {
    Point2 lm = results.at<Point2>(L(i));
    lm_outfile << lm.x() << " " << lm.y()
               << endl;  // WRONG FORMAT... JUST FOR SIMPLE SHOW
  }

  lm_outfile.close();
  cout << "output " << lm_file_name << " file." << endl;

  //*
  // Output each hypo
  ofstream outfile_hypo;
  string hypo_file_name = "HybridISAM_victoriaPark_hypos";
  outfile_hypo.open(hypo_file_name + ".txt");

  //*
  ofstream outfile_time;
  string time_file_name = "HybridISAM_victoriaPark_time";
  outfile_time.open(time_file_name + ".txt");
  for (auto acc_time : time_list) {
    outfile_time << acc_time << endl;
  }
  outfile_time.close();
  cout << "output " << time_file_name << ".txt file." << endl;
  // */

  return 0;
}
