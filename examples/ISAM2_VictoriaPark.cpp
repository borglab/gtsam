#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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

using symbol_shorthand::L;
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
const size_t max_odom_count = 6967;  // 3800

const bool is_with_ambiguity = false;  // run original iSAM2 without ambiguities
// const bool is_with_ambiguity = true;  // run original iSAM2 with ambiguities

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
  ifstream in(findExampleDataFile("victoriaPark_01.txt"));

  size_t odom_count = 0;
  size_t landmark_count = 0;

  std::list<double> time_list;

  ISAM2Params parameters;

  parameters.optimizationParams =
      gtsam::ISAM2GaussNewtonParams(0.0);  //_wildfireThreshold = 0.001

  parameters.relinearizeThreshold = 0.01;

  parameters.relinearizeSkip = 1;
  ISAM2* isam2 = new ISAM2(parameters);

  NonlinearFactorGraph* graph = new NonlinearFactorGraph();

  Values init_values;
  Values results;

  double x = 0.0;
  double y = 0.0;
  double rad = 0.0;

  Pose2 prior_pose(x, y, rad);

  init_values.insert(X(0), prior_pose);

  graph->add(PriorFactor<Pose2>(X(0), prior_pose, prior_noise_model));

  isam2->update(*graph, init_values);
  graph->resize(0);
  init_values.clear();
  results = isam2->calculateBestEstimate();

  //*
  size_t key_s = 0;
  size_t key_t = 0;

  clock_t start_time = clock();
  string str;
  while (getline(in, str) && odom_count < max_odom_count) {
    // cout << str << endl;
    vector<string> parts;
    split(parts, str, is_any_of(" "));

    if (parts[0] == "ODOMETRY") {
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

      Pose2 odom_pose;
      if (is_with_ambiguity) {
        // Get wrong intentionally
        int id = odom_count % m_num;
        odom_pose = Pose2(x_arr[id], y_arr[id], rad_arr[id]);
      } else {
        odom_pose = Pose2(x_arr[0], y_arr[0], rad_arr[0]);
      }

      init_values.insert(X(key_t), results.at<Pose2>(X(key_s)) * odom_pose);

      graph->add(BetweenFactor<Pose2>(X(key_s), X(key_t), odom_pose,
                                      pose_noise_model));

      odom_count++;
      std::cout << "odom_count: " << odom_count << std::endl;

    } else {  // LANDMARK
      //*
      key_s = stoi(parts[1]);
      int k_num = stoi(parts[2]);

      if (k_num == 1) {
        key_t = stoi(parts[3]);
        // int m_num = 1;
        double l_x = stod(parts[6]);
        double l_y = stod(parts[7]);

        Point2 measured_point(l_x, l_y);

        if (key_t >= landmark_count) {
          init_values.insert(L(key_t), results.at<Pose2>(X(key_s)) *
                                           measured_point);  // operator*
          landmark_count++;
        }

        graph->add(Pose2Point2Factor(X(key_s), L(key_t), measured_point,
                                     point_noise_model));

      } else {
        if (is_with_ambiguity) {
          // Get wrong intentionally
          if (odom_count % 4 != 0) {
            key_t = stoi(parts[4]);
          } else {
            key_t = stoi(parts[3]);
          }

        } else {
          key_t = stoi(parts[3]);
        }

        double l_x = stod(parts[7]);
        double l_y = stod(parts[8]);

        Point2 measured_point(l_x, l_y);

        graph->add(Pose2Point2Factor(X(key_s), L(key_t), measured_point,
                                     point_noise_model));
      }
    }

    isam2->update(*graph, init_values);
    graph->resize(0);
    init_values.clear();
    results = isam2->calculateBestEstimate();

    if (parts[0] == "ODOMETRY") {
      clock_t cur_time = clock();
      time_list.push_back(cur_time - start_time);
    }
  }

  clock_t end_time = clock();
  clock_t total_time = end_time - start_time;
  cout << "total_time: " << total_time << endl;
  //*

  ofstream outfile;
  string file_name = "ISAM2_victoriaPark.txt";
  outfile.open(file_name);

  for (size_t i = 0; i < odom_count; ++i) {
    Pose2 out_pose = results.at<Pose2>(X(i));

    outfile << out_pose.x() << " " << out_pose.y() << " " << out_pose.theta()
            << endl;
  }
  outfile.close();
  cout << "output " << file_name << " file." << endl;
  // */
  ofstream lm_outfile;
  string lm_file_name = "ISAM2_victoriaPark_lm.txt";
  lm_outfile.open(lm_file_name);

  for (size_t i = 0; i < landmark_count; ++i) {
    Point2 out_point = results.at<Point2>(L(i));

    lm_outfile << out_point.x() << " " << out_point.y() << endl;
  }
  lm_outfile.close();
  cout << "output " << lm_file_name << " file." << endl;

  //*
  ofstream outfile_time;
  string time_file_name = "ISAM2_victoriaPark_time.txt";
  outfile_time.open(time_file_name);
  for (auto acc_time : time_list) {
    outfile_time << acc_time << endl;  // WRONG FORMAT... JUST FOR SIMPLE SHOW
  }
  outfile_time.close();
  cout << "output " << time_file_name << " file." << endl;
  // */
  return 0;
}
