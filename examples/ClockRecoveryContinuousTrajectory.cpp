
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/basis/polynomial/TrajectoryModel.h>
#include <gtsam/basis/polynomial/IrwinHall.h>
#include <gtsam/geometry/Pose3.h>

/**
* In this example, we estimate pose from pictures and landmarks using bundle adjustment in the usual way
* we also capture acceleration and rotation rates with a mems IMU on the camera.
* Notably, we don't assume that the camera, accelerometer, or gryo clocks are related to each other.
* We pick one sensor to be the clock datum, the other sensors' clocks are modelled as drifting in
* relation to that datum.
* we'll choose the camera's frame rate to be the datum clock for this example.
*/


// structs to collect and name the important measurements from each sensor
struct {
  gtsam::Key camera_pose;  // allow bundle adjustment to associate a Pose3 with a Key
  double timestamp;  // the timestamp (in seconds) for the shutter based on the camera's frame rate.
} shutter_event_t;

struct {
  Vector3 acceleration;  // raw acceleration measurement
  double timestamp;  // the timestamp (in seconds) extrapolated from the accelerometer sample rate
} acceleration_t;

struct {
  Vector3 angular_velocity; // raw angular velocity measurement
  double timestamp;  // the timestamp (in seconds) extrapolated from the gyro sample rate
} angular_rate_t;



// alias the time derivatives of pose so we can express intent through types
using Vector3_ = Expression<Vector3>;
using Vector6_ = Expression<Vector6>;

using Pose3Deriv_ = Expression<typename traits<Pose3>::TangentVector>; // Vector6_
using Pose3Vel_ = Pose3Deriv_;
using Pose3Accel_ = Pose3Deriv_;

using Point3Deriv_ = Vector3_;
using Point3Accel_ = Point3Deriv_;

using Rot3Deriv_ = Vector3_;
using Rot3Vel_ = Rot3Deriv_;



// We need to extract rotation and translation components from time derivatives of Pose3
Rot3Deriv_ pose3DerivativeRotation(Pose3Deriv_ pose_derivative, OptionalJacobian<3, 6> H = {}){
  if (H) *H << I_3x3, Z_3x3;
  return pose_derivative.seq(0,2);
}
Point3Deriv_ pose3DerivativeTranslation(Pose3Deriv_ pose_derivative, OptionalJacobian<3, 6> H = {}){
  if (H) *H << Z_3x3, I_3x3;
  return pose_derivative.seq(3,5);
}
inline Rot3Deriv_ pose3DerivativeRotation(const Pose3Deriv_& p) {
  return Vector3_(&pose3DerivativeRotation, p);
}
inline Point3Deriv_ pose3DerivativeTranslation(const Pose3Deriv_& p) {
  return Point3Deriv_(&pose3DerivativeTranslation, p);
}

// we also need to compose Pose3 TangentVectors with Pose3s
Pose3Deriv_ adjoint(const Pos3_& p, const Pose3Deriv_& v)
{
  return Point3_(p, &Pose3::Adjoint, v);
}

// enum to make the sampleTrajectoryDerivative arguemnts obvious
struct TimeDerivatives{
  enum TimeDerivatives{
    velocity = 1;
    acceleration = 2;
    jerk = 3;
    snap = 4;
    crackle = 5;
    pop = 6;
  }
};



void simulate_run()
{

  gtsam::Values initial_values;

  // model the trajectory with cubic splines (makes accelerations piecewise linear)
  TrajectoryModel<Pose3> trajectory_model(kernels::IrwinHallCDF2);
  // model the clock drift with a quadratic spline, (makes drift rate piecewise linear)
  TrajectoryModel<double> accel_clock_model(kernels::IrwinHallCDF1);
  TrajectoryModel<double> gyro_clock_model(kernels::IrwinHallCDF1);

  // consider the bandwidth you need to represent the dynamics you want to model
  // any sensor bandwidth above this frequency contributes to graph noise.
  double trajectory_sample_rate = 20; // control points per second
  double clock_drift_sample_rate = 0.1; // control points per second


  // Account for typical raw-sensor weirdness:

  // we know the IMU is not located at the camera's optical centre, we can compensate that.
  // If these were Keys, we could estimate the location of the IMU relative to the camera.
  Pose3_ accel_pose = Pose3_(Pose3());
  Pose3_ gyro_pose = Pose3_(Pose3());

  // Gravity vector gives us a massive hint about orientations, so keep it in our model
  Point3Accel_ gravity = Point3Accel_(Vector3(0,0,9.81)); // North East Down coords

  // the raw sensor will have a small bias to it, constrain the magnitude with a prior.
  // for extremely long runs with thermal drift, these can also be a trajectory
  Key accel_bias_key = Key('b', 0);
  Key gyro_bias_key = Key('b', 1);
  Point3Accel_ accel_bias = Point3Accel_(accel_bias_key);
  Rot3Vel_ gyro_bias = Rot3Vel_(gyro_bias_key);
  initial_values.insert<Vector3>(accel_bias_key, Vector3(0,0,0));
  initial_values.insert<Vector3>(gyro_bias_key, Vector3(0,0,0));



  // we'll build a bunch of trajectory up front, this can be done incrementally the same way
  // just make sure that the trajectory is longer than your new timestamp + the kernel length before generating expressions.
  double run_length = 10.0; //seconds
  // add some control points for poses
  for(int i=0;i< ceil(run_length * trajectory_sample_rate); i++)
  {
    Key pose_key = Key('p', i);
    trajectory_model.add_control_point(Pose3_(pose_key));
    initial_values.insert<Pose3>(pose_key, Pose3());
  }

  // add control points for clock drift
  for(int i=0;i< ceil(run_length * clock_drift_sample_rate); i++)
  {
    Key accel_clock_drift_key = Key('a', i);
    Key gyro_clock_drift_key = Key('g', i);
    initial_values.insert<double>(accel_drift_key, 0.0);
    initial_values.insert<double>(gyro_clock_drift_key, 0.0);
    accel_clock_model.add_control_point(Pose3_(accel_clock_drift_key));
    gyro_clock_model.add_control_point(Pose3_(gyro_clock_drift_key));
  }




  // sample each sensor
  shutter_event_t shutter_event = sample_camera_add_slam_factors();
  acceleration_t acceleration = sample_accelerometer();
  angular_rate_t angular_rate = sample_gyro();

  // use the clock models to estimate the sensor clock's offset from the datum for a given timestamp
  // being sure to scale the timestamp from seconds to units of control-points before sampling the trajectory
  Double_ gyro_clock_offset = gyro_clock_model.sampleTrajectory(Double_(angular_rate.timestamp * clock_drift_sample_rate));
  Double_ accel_clock_offset = accel_clock_model.sampleTrajectory(Double_(acceleration.timestamp * clock_drift_sample_rate));

  // scale the timestamps from seconds into units of control_points and apply clock offsets
  Double_ camera_time = Double_(shutter_event.timestamp * trajectory_sample_rate);
  Double_ gyro_time = (Double_(angular_rate.timestamp) - gyro_clock_offset) * trajectory_sample_rate;
  Double_ accel_time = (Double_(acceleration.timestamp) - accel_clock_offset) * trajectory_sample_rate;



  // start evaluating the pose trajectory model

  // we can prune control points out of the Expression by setting the window around the timestamp
  double window_start = 0;  // timestamp - temporal_uncertainty
  double window_end = -1; // timestamp + temporal_uncertainty


  // construct an expression that estimates the pose at the shutter time
  Pose3_ camera_pose = trajectory_model.sampleTrajectory(camera_time, window_start, window_end);
  // construct an expression that estimates the velocity at the gyro sample time
  Pose3Vel_ camera_velocity = trajectory_model.sampleTrajectoryDerivative(gyro_time, window_start, window_end, TimeDerivatives::velocity);
  // construct an expression that estimates the acceleration at the accelerometer sample time
  Pose3Accel_ camera_acceleration = trajectory_model.sampleTrajectoryDerivative(accel_time, window_start, window_end, TimeDerivatives::acceleration);


  // transform the trajectory velocity and acceleration predictions into the sensor's local coords
  // XXX I'm not 100% sure that this is the right way of transforming the TangentVector,
  //     this might need accel_pose.inverse().adjoint()
  Pose3Vel_ gyro_vel = gyro_pose.adjoint(camera_velocity);
  Pose3Accel_ accel_accel = accel_pose.adjoint(camera_acceleration);


  // construct the pose factors connecting the trajectory_model to the slam bundle-adjustment graph
  auto camera_factor = ExpressionFactor<Pose3>(
    /*noise model*/
    Pose3(),  // identity transform
    between(camera_pose, Pose3_(shutter_event.camera_pose)), // discrepancy between bundle adjustment and trajectory model
  );

  // construct the velocity factor
  auto gyro_factor = ExpressionFactor<Vector3>(
    /*raw sensor noise model*/
    angular_rate.angular_velocity,
    Pose3DerivativeRotation(gyro_vel) + gyro_bias
  );

  // construct the acceleration factor
  auto accel_factor = ExpressionFactor<Vector3>(
    /*raw sensor noise model*/
    acceleration.acceleration,
    Pose3DerivativeTranslation(accel_accel) + gravity + accel_bias
  );




  // We can also constrain the smoothness of the trajectory by adding factors to high derivatives
  // Factors are only needed at the control point rate.
  // This is equivalent to certain forms of Penalty Spline methods
  for(int i=0; i<trajectory_model.getControlPoints().size(); i++){
    auto smoothness_factor = ExpressionFactor<Vector6>(
      /* any robust noise model */
      Vector6()::Zero(),
      trajectory_model.sampleTrajectoryDerivative(i, window_start, window_end, TimeDerivatives::crackle);
    );
  }


}


