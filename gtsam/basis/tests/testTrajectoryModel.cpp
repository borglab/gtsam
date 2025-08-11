/**
 * @file    TestTrajectoryModel.cpp
 * @brief   unit tests for cardinal spline interpolation
 * @author  Brett Downing
 * @date    August 2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/basis/polynomial/TrajectoryModel.h>
#include <gtsam/basis/polynomial/IrwinHall.h>

using namespace gtsam;



// Lie comopsitions stay valid for long tangent traversals,
// tolerance should accomodate second-order effects
double epsilon = 1e-4;
double tolerance = epsilon/4.0;


TEST( TrajectoryModel , BasicBounds ) {
  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<Double_> path({
    Double_(0.0),
    Double_(1.0),
    Double_(0.0),
    Double_(1.0),
  });

  // rear padding by default
  // bool pad_front = false;
  // with padding at the back,
  //   non-constant timestamps will be in
  //   [0 : path.size()-1 + basis_function.get_length()]
  TrajectoryModel<double> model(basis_function, path/*, pad_front*/);

  Key t_key = Key(0); // choose a key to carry the timestamp
  v.insert<double>(t_key, 0.0); // assign a value to the timestamp
  Double_ timestamp = Double_(t_key); // associate the timestamp with the key

  // construct an Expression that samples the trajectory
  Double_ sample = model.sample_trajectory(timestamp);


  // bounds check
  v.update<double>(t_key, 0);
  CHECK(assert_equal(path.front().value(v), sample.value(v), tolerance));

  v.update<double>(t_key, path.size()-1 + basis_function.get_length());
  CHECK(assert_equal(path.back().value(v), sample.value(v), tolerance));

  // mid value
  v.update<double>(t_key, (path.size() + basis_function.get_length())/2);
  CHECK(assert_equal((path.front().value(v)+path.back().value(v))/2, sample.value(v), tolerance));

}



TEST( TrajectoryModel , FrontPadding ) {
  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<Double_> path({
    Double_(0.0),
    Double_(1.0),
    Double_(0.0),
    Double_(1.0),
  });
  bool pad_front = true;
  // with padding at the front,
  //   non-constant timestamps will be in
  //   [-basis_function.get_length() : path.size()-1]
  TrajectoryModel<double> model(basis_function, path, pad_front);

  Key t_key = Key(0); // choose a key to carry the timestamp
  v.insert<double>(t_key, 0.0); // assign a value to the timestamp
  Double_ timestamp = Double_(t_key); // associate the timestamp with the key

  // construct an Expression that samples the trajectory
  Double_ sample = model.sample_trajectory(timestamp);


  // bounds check
  v.update<double>(t_key, -basis_function.get_length());
  CHECK(assert_equal(path.front().value(v), sample.value(v), tolerance));

  v.update<double>(t_key, path.size()-1);
  CHECK(assert_equal(path.back().value(v), sample.value(v), tolerance));

  // mid value
  v.update<double>(t_key, (-basis_function.get_length() + (path.size()))/2);
  CHECK(assert_equal((path.front().value(v)+path.back().value(v))/2, sample.value(v), tolerance));

}





TEST( TrajectoryModel , WindowTruncation ) {
  // TrajectoryModel can interpolate within a range of control points
  // bringing the compute cost down to O(window_size).
  // the result should be the same as long as enough points are included
  // before or after the window.
  // padding is automatically applied to the window arguments

  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<Double_> path({
    Double_(4.0),
    Double_(-4.0),
    Double_(3.0),
    Double_(7.0),

    Double_(3.0),
    Double_(-8.0),
    Double_(2.0),
    Double_(6.0),

    Double_(4.0),
    Double_(1.0),
    Double_(3.0),
    Double_(-2.0),

    Double_(5.0),
    Double_(2.0),
    Double_(9.0),
    Double_(-2.0)
  });

  //rear padding by default
  TrajectoryModel<double> model(basis_function, path);

  Key t_key = Key(0); // choose a key to carry the timestamp
  v.insert<double>(t_key, 0.0); // assign a value to the timestamp
  Double_ timestamp = Double_(t_key); // associate the timestamp with the key

  // construct an Expression that samples from the entire trajectory
  Double_ sample = model.sample_trajectory(timestamp);

  // construct an Expression that samples from a subset of the trajectory
  double window_start = 6;
  double window_end = 8;
  Double_ trunc_back = model.sample_trajectory(timestamp, 0, window_end);
  Double_ trunc_front = model.sample_trajectory(timestamp, window_start, -1);
  Double_ trunc_both = model.sample_trajectory(timestamp, window_start, window_end);

  for(double t=window_start; t<window_end; t+=0.1)
  {
    v.update<double>(t_key, t);
    CHECK(assert_equal(sample.value(v), trunc_back.value(v), tolerance));
    CHECK(assert_equal(sample.value(v), trunc_front.value(v), tolerance));
    CHECK(assert_equal(sample.value(v), trunc_both.value(v), tolerance));

  }
}



TEST( TrajectoryModel , TypesPose3 ) {
  // tests TrajectoryModel can be constructed for Pose3
  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<Pose3_> path({
    Pose3_(Pose3(Rot3(), Point3(0,0,0))),
    Pose3_(Pose3(Rot3(), Point3(0,0,0))),
    Pose3_(Pose3(Rot3(), Point3(0,2,0))),
    Pose3_(Pose3(Rot3(), Point3(0,2,0))),
  });

  //rear padding by default
  TrajectoryModel<Pose3> model(basis_function, path);

  Key t_key = Key(0); // choose a key to carry the timestamp
  v.insert<double>(t_key, 0.0); // assign a value to the timestamp
  Double_ timestamp = Double_(t_key); // associate the timestamp with the key

  // construct an Expression that samples from the entire trajectory
  Pose3_ sample = model.sample_trajectory(timestamp);

  v.update<double>(t_key, 3.5);
  CHECK(assert_equal(Pose3(Rot3(), Point3(0,1,0)), sample.value(v), tolerance));

}





TEST( TrajectoryModel , DerivativePose3 ) {
  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<Pose3_> path({
    Pose3_(Pose3(Rot3::Rodrigues(0.3,2.2,0.1), Point3(0,0,0))),
    Pose3_(Pose3(Rot3::Rodrigues(0.2,2.5,0.1), Point3(0,0,0))),
    Pose3_(Pose3(Rot3::Rodrigues(0.0,2.2,0.1), Point3(0,2,0))),
    Pose3_(Pose3(Rot3::Rodrigues(0.2,2.1,0.4), Point3(0,2,0))),
  });

  TrajectoryModel<Pose3> model(basis_function, path);

  Key t_ref_key = Key(0); // key to carry the timestamp
  Key t_eps_key = Key(1); // key to carry the timestamp + epsilon

  // assign types to Keys
  v.insert<double>(t_ref_key, 0.0);
  v.insert<double>(t_eps_key, 0.0);

  // cast the Keys to Expressions
  Double_ t_ref = Double_(t_ref_key);
  Double_ t_eps = Double_(t_eps_key);

  // construct Expressions that sample the trajectory
  Pose3_ sample_ref = model.sample_trajectory(t_ref);
  Pose3_ sample_eps = model.sample_trajectory(t_eps);
  // construct an expression that captures the tangent
  auto sample_d = model.sample_trajectory_d(t_ref,0,-1,1);

  for(double sample_time = 0; sample_time<7; sample_time+=0.1)
  {
    v.update<double>(t_ref_key, sample_time);
    v.update<double>(t_eps_key, sample_time + epsilon);
    CHECK(assert_equal(
      sample_eps.value(v),
      //sample_ref.value(v),
      expmap(sample_ref, epsilon * sample_d).value(v),
      tolerance
    ));
  }

}



TEST( TrajectoryModel , NthDerivativePose3 ) {
  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<Pose3_> path({
    Pose3_(Pose3(Rot3(), Point3(0,0,0))), // XXX use non-trival rotations
    Pose3_(Pose3(Rot3(), Point3(0,0,0))),
    Pose3_(Pose3(Rot3(), Point3(0,2,0))),
    Pose3_(Pose3(Rot3(), Point3(0,2,0))),
  });

  TrajectoryModel<Pose3> model(basis_function, path);

  Key t_ref_key = Key(0); // key to carry the timestamp
  Key t_eps_key = Key(1); // key to carry the timestamp + epsilon

  // assign types to Keys
  v.insert<double>(t_ref_key, 0.0);
  v.insert<double>(t_eps_key, 0.0);

  // cast the Keys to Expressions
  Double_ t_ref = Double_(t_ref_key);
  Double_ t_eps = Double_(t_eps_key);

  for(size_t n=0; n<basis_function.get_valid_derivatives()-1; n++)
  {
    // construct Expressions that sample the Nth derivative of the trajectory
    auto sample_ref = model.sample_trajectory_d(t_ref, 0, -1, n);
    auto sample_eps = model.sample_trajectory_d(t_eps, 0, -1, n);
    // construct an Expression that captures the (N+1)th derivative of the trajectory
    auto sample_d = model.sample_trajectory_d(t_ref,0,-1, n+1);
    for(double sample_time = 0; sample_time<7; sample_time+=0.1)
    {
      v.update<double>(t_ref_key, sample_time);
      v.update<double>(t_eps_key, sample_time + epsilon);
      CHECK(assert_equal(
        sample_eps.value(v),
        expmap(sample_ref, epsilon * sample_d).value(v),
        tolerance
      ));
    }
  }

}



TEST( TrajectoryModel , MeshModel ) {
  // TrajectoryModel accepts Expressions as control points,
  // allowing arbitrarily complex control point definitions,
  // such as entire other TrajectoryModel samples

  gtsam::Values v;  // needed for evaluating Expressions

  // choose a cubic spline
  const kernel_base& basis_function = kernels::IrwinHallCDF2;

  // specify some control points
  std::vector<std::vector<Double_>> mesh({
    {
      Double_(4.0),
      Double_(-4.0),
      Double_(3.0),
      Double_(7.0),
    },
    {
      Double_(3.0),
      Double_(-8.0),
      Double_(2.0),
      Double_(6.0),
    },
    {
      Double_(4.0),
      Double_(1.0),
      Double_(3.0),
      Double_(-2.0),
    },
    {

      Double_(5.0),
      Double_(2.0),
      Double_(9.0),
      Double_(-2.0)
    }
  });
  

  Key x_key = Key(0);
  Key y_key = Key(1);
  v.insert<double>(x_key, 0.0);
  v.insert<double>(y_key, 0.0);
  Double_ x_expr(x_key);
  Double_ y_expr(y_key);

  std::vector<Double_> col_path;
  std::vector<TrajectoryModel<double>> rows;

  for(const auto& path : mesh){
    TrajectoryModel<double> row(basis_function, path);
    rows.push_back(row);
    auto row_sample = row.sample_trajectory(x_expr);
    col_path.push_back(row_sample);
  }
  TrajectoryModel<double> col(basis_function, col_path);
  auto sample = col.sample_trajectory(y_expr);


  for(double x=0; x<7; x+=0.1)
  {
    for(double y=0; y<7; y+=0.1)
    {
      v.update<double>(x_key, x);
      v.update<double>(y_key, y);
      CHECK(sample.value(v)<10);
      CHECK(sample.value(v)>-10);
    }
  }
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

