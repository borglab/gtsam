/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testShonanAveraging.cpp
 * @date   September 2019
 * @author Jing Wu
 * @author Frank Dellaert
 * @brief  Timing script for Shonan Averaging algorithm
 */

#include "gtsam/base/Matrix.h"
#include "gtsam/base/Vector.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Rot3.h"
#include <gtsam/base/timing.h>
#include <gtsam/sfm/ShonanAveraging.h>

#include <CppUnitLite/TestHarness.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>

using namespace std;
using namespace gtsam;

// save a single line of timing info to an output stream
void saveData(size_t p, double time1, double costP, double cost3, double time2,
              double min_eigenvalue, double suBound, std::ostream* os) {
  *os << static_cast<int>(p) << "\t" << time1 << "\t" << costP << "\t" << cost3
      << "\t" << time2 << "\t" << min_eigenvalue << "\t" << suBound << endl;
}

void checkR(const Matrix& R) {
    Matrix R2 = R.transpose();
    Matrix actual_R = R2 * R;
    assert_equal(Rot3(), Rot3(actual_R));
}

void saveResult(string name, const Values& values) {
    ofstream myfile;
    myfile.open("shonan_result_of_" + name + ".dat");
    size_t nrSO3 = values.count<SO3>();
    myfile << "#Type SO3 Number " << nrSO3 << "\n";
    for (size_t i = 0; i < nrSO3; ++i) {
        Matrix R = values.at<SO3>(i).matrix();
        // Check if the result of R.Transpose*R satisfy orthogonal constraint
        checkR(R);
        myfile << i;
        for (int m = 0; m < 3; ++m) {
            for (int n = 0; n < 3; ++n) {
                myfile << " " << R(m, n);
            }
        }
        myfile << "\n";
    }
    myfile.close();
    cout << "Saved shonan_result.dat file" << endl;
}

void saveG2oResult(string name, const Values& values, std::map<Key, Pose3> poses) {
    ofstream myfile;
    myfile.open("shonan_result_of_" + name + ".g2o");
    size_t nrSO3 = values.count<SO3>();
    for (size_t i = 0; i < nrSO3; ++i) {
        Matrix R = values.at<SO3>(i).matrix();
        // Check if the result of R.Transpose*R satisfy orthogonal constraint
        checkR(R);
        myfile << "VERTEX_SE3:QUAT" << " ";
        myfile << i << " ";
        myfile << poses[i].x() << " " << poses[i].y() << " " << poses[i].z() << " ";
        Vector quaternion = Rot3(R).quaternion();
        myfile << quaternion(3) << " " << quaternion(2) << " " << quaternion(1)
               << " " << quaternion(0);
        myfile << "\n";
    }
    myfile.close();
    cout << "Saved shonan_result.g2o file" << endl;
}

void saveResultQuat(const Values& values) {
    ofstream myfile;
    myfile.open("shonan_result.dat");
    size_t nrSOn = values.count<SOn>();
    for (size_t i = 0; i < nrSOn; ++i) {
        GTSAM_PRINT(values.at<SOn>(i));
        Rot3 R = Rot3(values.at<SOn>(i).matrix());
        float x = R.toQuaternion().x();
        float y = R.toQuaternion().y();
        float z = R.toQuaternion().z();
        float w = R.toQuaternion().w();
        myfile << "QuatSO3 " << i;
        myfile << "QuatSO3 " << i << " " << w << " " << x << " " << y << " " << z << "\n";
        myfile.close();
    }
}

int main(int argc, char* argv[]) {
    // primitive argument parsing:
    if (argc > 3) {
        throw runtime_error("Usage: timeShonanAveraging [g2oFile]");
    }

    string g2oFile;
    try {
      if (argc > 1)
        g2oFile = argv[argc - 1];
      else
        g2oFile = findExampleDataFile("sphere2500");

    } catch (const exception& e) {
      cerr << e.what() << '\n';
      exit(1);
    }

    // Create a csv output file
    size_t pos1 = g2oFile.find("data/");
    size_t pos2 = g2oFile.find(".g2o");
    string name = g2oFile.substr(pos1 + 5, pos2 - pos1 - 5);
    cout << name << endl;
    ofstream csvFile("shonan_timing_of_" + name + ".csv");

    // Create Shonan averaging instance from the file.
    // ShonanAveragingParameters parameters;
    // double sigmaNoiseInRadians = 0 * M_PI / 180;
    // parameters.setNoiseSigma(sigmaNoiseInRadians);
    static const ShonanAveraging3 kShonan(g2oFile);

    // increase p value and try optimize using Shonan Algorithm. use chrono for
    // timing
    size_t pMin = 3;
    Values Qstar;
    Vector minEigenVector;
    double CostP = 0, Cost3 = 0, lambdaMin = 0, suBound = 0;
    cout << "(int)p" << "\t" << "time1" << "\t" << "costP" << "\t" << "cost3" << "\t"
        << "time2" << "\t" << "MinEigenvalue" << "\t" << "SuBound" << endl;

    const Values randomRotations = kShonan.initializeRandomly();

    for (size_t p = pMin; p <= 7; p++) {
        // Randomly initialize at lowest level, initialize by line search after that
        const Values initial =
            (p > pMin) ? kShonan.initializeWithDescent(p, Qstar, minEigenVector,
                                                       lambdaMin)
                       : ShonanAveraging3::LiftTo<Rot3>(pMin, randomRotations);
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        // optimizing
        const Values result = kShonan.tryOptimizingAt(p, initial);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> timeUsed1 =
            chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        lambdaMin = kShonan.computeMinEigenValue(result, &minEigenVector);
        chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
        chrono::duration<double> timeUsed2 =
            chrono::duration_cast<chrono::duration<double>>(t3 - t1);
        Qstar = result;
        CostP = kShonan.costAt(p, result);
        const Values SO3Values = kShonan.roundSolution(result);
        Cost3 = kShonan.cost(SO3Values);
        suBound = (Cost3 - CostP) / CostP;

        saveData(p, timeUsed1.count(), CostP, Cost3, timeUsed2.count(),
                 lambdaMin, suBound, &cout);
        saveData(p, timeUsed1.count(), CostP, Cost3, timeUsed2.count(),
                 lambdaMin, suBound, &csvFile);
    }
    saveResult(name, kShonan.roundSolution(Qstar));
    // saveG2oResult(name, kShonan.roundSolution(Qstar), kShonan.Poses());
    return 0;
}
