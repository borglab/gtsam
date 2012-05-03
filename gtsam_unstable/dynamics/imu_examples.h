/**
 * @file imu_examples.h
 * @brief Example measurements from ACFR matlab simulation
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

namespace gtsam {
namespace examples {

// examples pulled from simulated dataset

// case pulled from dataset (frame 5000, no noise, flying robot)
namespace frame5000 {
double dt=0.010000;

// previous frame
gtsam::Point3 posInit(
		-34.959663877411039,
		-36.312388041359441,
		-9.929634578582256);
gtsam::Vector velInit = gtsam::Vector_(3,
		-2.325950469365050,
		-5.545469040035986,
		0.026939998635121);
gtsam::Vector attInit = gtsam::Vector_(3,
		-0.005702574137157,
		-0.029956547314287,
		1.625011216344428);

// IMU measurement (accel (0-2), gyro (3-5)) - from current frame
gtsam::Vector accel = gtsam::Vector_(3,
		1.188806676070333,
	 -0.183885870345845,
	 -9.870747316422888);
gtsam::Vector gyro  = gtsam::Vector_(3,
		0.0,
		0.026142019158168,
	 -2.617993877991494);

// resulting frame
gtsam::Point3 posFinal(
		-34.982904302954310,
		-36.367693859767584,
		-9.929367164045985);
gtsam::Vector velFinal = gtsam::Vector_(3,
		-2.324042554327658,
		-5.530581840815272,
		0.026741453627181);
gtsam::Vector attFinal = gtsam::Vector_(3,
		-0.004918046965288,
		-0.029844423605949,
		1.598818460739227);

PoseRTV init (posInit, gtsam::Rot3::RzRyRx(attInit), velInit);
PoseRTV final(posFinal, gtsam::Rot3::RzRyRx(attFinal), velFinal);
}

// case pulled from dataset (frame 10000, no noise, flying robot)
namespace frame10000 {
double dt=0.010000;

// previous frame
gtsam::Point3 posInit(
		-30.320731549352189,
		-1.988742283760434,
		-9.946212692656349);
gtsam::Vector velInit = gtsam::Vector_(3,
		-0.094887047280235,
		-5.200243574047883,
		-0.006106911050672);
gtsam::Vector attInit = gtsam::Vector_(3,
		-0.049884854704728,
		-0.004630595995732,
		-1.952691683647753);

// IMU measurement (accel (0-2), gyro (3-5)) - from current frame
gtsam::Vector accel = gtsam::Vector_(3,
		0.127512027192321,
		0.508566822495083,
	 -9.987963604829643);
gtsam::Vector gyro  = gtsam::Vector_(3,
		0.0,
		0.004040957322961,
		2.617993877991494);

// resulting frame
gtsam::Point3 posFinal(
		-30.321685191305157,
		-2.040760054006146,
		-9.946292804391417);
gtsam::Vector velFinal = gtsam::Vector_(3,
		-0.095364195297074,
		-5.201777024575911,
		-0.008011173506779);
gtsam::Vector attFinal = gtsam::Vector_(3,
		-0.050005924151669,
		-0.003284795837998,
		-1.926546047162884);

PoseRTV init (posInit, gtsam::Rot3::RzRyRx(attInit), velInit);
PoseRTV final(posFinal, gtsam::Rot3::RzRyRx(attFinal), velFinal);
}

// case pulled from dataset (frame at time 4.00, no noise, flying robot, poses from 3.99 to 4.0)
namespace flying400 {
double dt=0.010000;

// start pose
// time 3.9900000e+00
// pos (x,y,z) 1.8226188e+01  -6.7168156e+01  -9.8236334e+00
// vel (dx,dy,dz) -5.2887267e+00   1.6117880e-01  -2.2665072e-02
// att (r, p, y) 1.0928413e-02  -2.0811771e-02   2.7206011e+00

// previous frame
gtsam::Point3 posInit(
		1.8226188e+01,  -6.7168156e+01,  -9.8236334e+00);
gtsam::Vector velInit = gtsam::Vector_(3,-5.2887267e+00,   1.6117880e-01,  -2.2665072e-02);
gtsam::Vector attInit = gtsam::Vector_(3, 1.0928413e-02,  -2.0811771e-02,   2.7206011e+00);

// time 4.0000000e+00
// accel 3.4021509e-01  -3.4413998e-01  -9.8822495e+00
// gyro  0.0000000e+00   1.8161697e-02  -2.6179939e+00

// IMU measurement (accel (0-2), gyro (3-5)) - ax, ay, az, r, p, y
gtsam::Vector accel = gtsam::Vector_(3,
		3.4021509e-01,  -3.4413998e-01,  -9.8822495e+00);
gtsam::Vector gyro  = gtsam::Vector_(3,
		0.0000000e+00,   1.8161697e-02,  -2.6179939e+00); // original version (possibly r, p, y)

// final pose
// time 4.0000000e+00
// pos (x,y,z) 1.8173262e+01  -6.7166500e+01  -9.8238667e+00
// vel (vx,vy,vz) -5.2926092e+00   1.6559974e-01  -2.3330881e-02
// att (r, p, y) 1.1473269e-02  -2.0344066e-02   2.6944191e+00

// resulting frame
gtsam::Point3 posFinal(1.8173262e+01,  -6.7166500e+01,  -9.8238667e+00);
gtsam::Vector velFinal = gtsam::Vector_(3,-5.2926092e+00,   1.6559974e-01,  -2.3330881e-02);
gtsam::Vector attFinal = gtsam::Vector_(3, 1.1473269e-02,  -2.0344066e-02,   2.6944191e+00);

// full states
PoseRTV init (posInit,  gtsam::Rot3::ypr( attInit(2),  attInit(1),  attInit(0)), velInit);
PoseRTV final(posFinal, gtsam::Rot3::ypr(attFinal(2), attFinal(1), attFinal(0)), velFinal);

} // \namespace flying400

namespace flying650 {
double dt=0.010000;

// from time 6.49 to 6.50 in robot F2 -

// frame (6.49)
// 6.4900000e+00  -3.8065039e+01  -6.4788024e+01  -9.8947445e+00  -5.0099274e+00   1.5999675e-01  -1.7762191e-02  -5.7708025e-03  -1.0109000e-02  -3.0465662e+00
gtsam::Point3 posInit(
		-3.8065039e+01,  -6.4788024e+01,  -9.8947445e+00);
gtsam::Vector velInit = gtsam::Vector_(3,-5.0099274e+00,   1.5999675e-01,  -1.7762191e-02);
gtsam::Vector attInit = gtsam::Vector_(3,-5.7708025e-03,  -1.0109000e-02,  -3.0465662e+00);

// IMU measurement (accel (0-2), gyro (3-5)) - ax, ay, az, r, p, y
// 6.5000000e+00  -9.2017256e-02   8.6770069e-02  -9.8213017e+00   0.0000000e+00   1.0728860e-02  -2.6179939e+00
gtsam::Vector accel = gtsam::Vector_(3,
		-9.2017256e-02,   8.6770069e-02,  -9.8213017e+00);
gtsam::Vector gyro  = gtsam::Vector_(3,
		 0.0000000e+00,   1.0728860e-02,  -2.6179939e+00); // in r,p,y

// resulting frame (6.50)
//  6.5000000e+00  -3.8115139e+01  -6.4786428e+01  -9.8949233e+00  -5.0099817e+00   1.5966531e-01  -1.7882777e-02  -5.5061386e-03  -1.0152792e-02  -3.0727477e+00
gtsam::Point3 posFinal(-3.8115139e+01,  -6.4786428e+01,  -9.8949233e+00);
gtsam::Vector velFinal = gtsam::Vector_(3,-5.0099817e+00,   1.5966531e-01,  -1.7882777e-02);
gtsam::Vector attFinal = gtsam::Vector_(3,-5.5061386e-03,  -1.0152792e-02,  -3.0727477e+00);

// full states
PoseRTV init (posInit,  gtsam::Rot3::ypr( attInit(2),  attInit(1),  attInit(0)), velInit);
PoseRTV final(posFinal, gtsam::Rot3::ypr(attFinal(2), attFinal(1), attFinal(0)), velFinal);

} // \namespace flying650


namespace flying39 {
double dt=0.010000;

// from time 0.38 to 0.39 in robot F1

// frame (0.38)
// 3.8000000e-01   3.4204706e+01  -6.7413834e+01  -9.9996055e+00  -2.2484370e-02  -1.3603911e-03   1.5267496e-02   7.9033358e-04  -1.4946656e-02  -3.1174147e+00
gtsam::Point3 posInit( 3.4204706e+01,  -6.7413834e+01,  -9.9996055e+00);
gtsam::Vector velInit = gtsam::Vector_(3,-2.2484370e-02,  -1.3603911e-03,   1.5267496e-02);
gtsam::Vector attInit = gtsam::Vector_(3, 7.9033358e-04,  -1.4946656e-02,  -3.1174147e+00);

// IMU measurement (accel (0-2), gyro (3-5)) - ax, ay, az, r, p, y
//    3.9000000e-01   7.2229967e-01  -9.5790214e-03  -9.3943087e+00   0.0000000e+00  -2.9157400e-01  -2.6179939e+00
gtsam::Vector accel = gtsam::Vector_(3, 7.2229967e-01,  -9.5790214e-03,  -9.3943087e+00);
gtsam::Vector gyro  = gtsam::Vector_(3, 0.0000000e+00,  -2.9157400e-01,  -2.6179939e+00); // in r,p,y

// resulting frame (0.39)
//  3.9000000e-01   3.4204392e+01  -6.7413848e+01  -9.9994098e+00  -3.1382246e-02  -1.3577532e-03   1.9568177e-02   1.1816996e-03  -1.7841704e-02   3.1395854e+00
gtsam::Point3 posFinal(3.4204392e+01,  -6.7413848e+01,  -9.9994098e+00);
gtsam::Vector velFinal = gtsam::Vector_(3,-3.1382246e-02,  -1.3577532e-03,   1.9568177e-02);
gtsam::Vector attFinal = gtsam::Vector_(3, 1.1816996e-03,  -1.7841704e-02,   3.1395854e+00);

// full states
PoseRTV init (posInit,  gtsam::Rot3::ypr( attInit(2),  attInit(1),  attInit(0)), velInit);
PoseRTV final(posFinal, gtsam::Rot3::ypr(attFinal(2), attFinal(1), attFinal(0)), velFinal);

} // \namespace flying39

namespace ground200 {
double dt=0.010000;

// from time 2.00 to 2.01 in robot G1

// frame (2.00)
// 2.0000000e+00   3.6863524e+01  -8.4874053e+01   0.0000000e+00  -7.9650687e-01   1.8345508e+00   0.0000000e+00   0.0000000e+00   0.0000000e+00   1.9804083e+00
gtsam::Point3 posInit(3.6863524e+01,  -8.4874053e+01,   0.0000000e+00);
gtsam::Vector velInit = gtsam::Vector_(3,-7.9650687e-01,   1.8345508e+00,   0.0000000e+00);
gtsam::Vector attInit = gtsam::Vector_(3, 0.0000000e+00,   0.0000000e+00,   1.9804083e+00);

// IMU measurement (accel (0-2), gyro (3-5)) - ax, ay, az, r, p, y
// 2.0100000e+00   1.0000000e+00   8.2156504e-15  -9.8100000e+00   0.0000000e+00   0.0000000e+00   0.0000000e+00
gtsam::Vector accel = gtsam::Vector_(3,
		1.0000000e+00,   8.2156504e-15,  -9.8100000e+00);
gtsam::Vector gyro  = gtsam::Vector_(3,
		0.0000000e+00,   0.0000000e+00,   0.0000000e+00); // in r,p,y

// resulting frame (2.01)
// 2.0100000e+00   3.6855519e+01  -8.4855615e+01   0.0000000e+00  -8.0048940e-01   1.8437236e+00   0.0000000e+00   0.0000000e+00   0.0000000e+00   1.9804083e+00
gtsam::Point3 posFinal(3.6855519e+01,  -8.4855615e+01,   0.0000000e+00);
gtsam::Vector velFinal = gtsam::Vector_(3,-8.0048940e-01,   1.8437236e+00,   0.0000000e+00);
gtsam::Vector attFinal = gtsam::Vector_(3, 0.0000000e+00,   0.0000000e+00,   1.9804083e+00);

// full states
PoseRTV init (posInit,  gtsam::Rot3::ypr( attInit(2),  attInit(1),  attInit(0)), velInit);
PoseRTV final(posFinal, gtsam::Rot3::ypr(attFinal(2), attFinal(1), attFinal(0)), velFinal);

} // \namespace ground200

namespace ground600 {
double dt=0.010000;

// from time 6.00 to 6.01 in robot G1

// frame (6.00)
// 6.0000000e+00   3.0320057e+01  -7.0883904e+01   0.0000000e+00  -4.0020377e+00   2.9972811e+00   0.0000000e+00   0.0000000e+00   0.0000000e+00   2.4987711e+00
gtsam::Point3 posInit(3.0320057e+01,  -7.0883904e+01,   0.0000000e+00);
gtsam::Vector velInit = gtsam::Vector_(3,-4.0020377e+00,   2.9972811e+00,   0.0000000e+00);
gtsam::Vector attInit = gtsam::Vector_(3, 0.0000000e+00,   0.0000000e+00,   2.4987711e+00);

// IMU measurement (accel (0-2), gyro (3-5)) - ax, ay, az, r, p, y
// 6.0100000e+00   6.1683759e-02   7.8536587e+00  -9.8100000e+00   0.0000000e+00   0.0000000e+00   1.5707963e+00
gtsam::Vector accel = gtsam::Vector_(3,
		6.1683759e-02,   7.8536587e+00,  -9.8100000e+00);
gtsam::Vector gyro  = gtsam::Vector_(3,
		0.0000000e+00,   0.0000000e+00,   1.5707963e+00); // in r,p,y

// resulting frame (6.01)
// 6.0100000e+00   3.0279571e+01  -7.0854564e+01   0.0000000e+00  -4.0486232e+00   2.9340501e+00   0.0000000e+00   0.0000000e+00   0.0000000e+00   2.5144791e+00
gtsam::Point3 posFinal(3.0279571e+01,  -7.0854564e+01,   0.0000000e+00);
gtsam::Vector velFinal = gtsam::Vector_(3,-4.0486232e+00,   2.9340501e+00,   0.0000000e+00);
gtsam::Vector attFinal = gtsam::Vector_(3, 0.0000000e+00,   0.0000000e+00,   2.5144791e+00);

// full states
PoseRTV init (posInit,  gtsam::Rot3::ypr( attInit(2),  attInit(1),  attInit(0)), velInit);
PoseRTV final(posFinal, gtsam::Rot3::ypr(attFinal(2), attFinal(1), attFinal(0)), velFinal);

} // \namespace ground600

} // \namespace examples
} // \namespace gtsam
