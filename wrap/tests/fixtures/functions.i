/**
 * A multi-line comment!
 */
// another comment

class gtsam::NonlinearFactorGraph;
class gtsam::Values;
class gtsam::noiseModel::Diagonal;

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename, Test* model, int maxID, bool addNoise, bool smart);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename, const gtsam::noiseModel::Diagonal* model, int maxID, bool addNoise, bool smart);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename, gtsam::noiseModel::Diagonal@ model);

Vector aGlobalFunction();

// An overloaded global function
Vector overloadedGlobalFunction(int a);
Vector overloadedGlobalFunction(int a, double b);

// A templated free/global function. Multiple templates supported.
template<T1 = {string, double}, T2 = {size_t}, R = {double}>
R MultiTemplatedFunction(const T& x, T2 y);

// Check if we can typedef the templated function
template<T>
void TemplatedFunction(const T& t);

typedef TemplatedFunction<gtsam::Rot3> TemplatedFunctionRot3;

// Check default arguments
void DefaultFuncInt(int a = 123, int b = 0);
void DefaultFuncString(const string& s = "hello", const string& name = "");
void DefaultFuncObj(const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter);
void DefaultFuncZero(int a = 0, int b, double c = 0.0, bool d = false, bool e);
void DefaultFuncVector(const std::vector<int> &i = {1, 2, 3}, const std::vector<string> &s = {"borglab", "gtsam"});

// Test for non-trivial default constructor
void setPose(const gtsam::Pose3& pose = gtsam::Pose3());
