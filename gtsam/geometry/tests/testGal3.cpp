/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testGal3.cpp
 * @brief   Unit tests for Gal3 class
 * @authors Matt Kielo, Scott Baker, Frank Dellaert
 * @date    April 29, 2025
 */

#include <gtsam/geometry/Gal3.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/testLie.h> // For CHECK_LIE_GROUP_DERIVATIVES, etc.
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h> // Included for kTestPose definition

#include <CppUnitLite/TestHarness.h>
#include <vector>
#include <functional> // For std::bind if using numerical derivatives

using namespace std;
using namespace gtsam;

// Define tolerance
static const double kTol = 1e-8;

// Instantiate Testable and Lie concepts for Gal3
GTSAM_CONCEPT_TESTABLE_INST(Gal3)
GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(Gal3)

// Define common test values
const Rot3 kTestRot = Rot3::RzRyRx(0.1, -0.2, 0.3);
const Point3 kTestPos(1.0, -2.0, 3.0);
const Velocity3 kTestVel(0.5, 0.6, -0.7);
const double kTestTime = 1.5;
const Pose3 kTestPose(kTestRot, kTestPos);
const Gal3 kTestGal3(kTestRot, kTestPos, kTestVel, kTestTime);


/* ************************************************************************* */
TEST(Gal3, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Gal3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Gal3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Gal3>);
}

/* ************************************************************************* */
// Define test instances for Lie group checks
const Gal3 kTestGal3_Lie1(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1.0, -2.0, 3.0), Velocity3(0.5, 0.6, -0.7), 1.5);
const Gal3 kTestGal3_Lie2(Rot3::RzRyRx(-0.2, 0.3, 0.1), Point3(-2.0, 3.0, 1.0), Velocity3(0.6, -0.7, 0.5), 2.0);
const Gal3 kIdentity_Lie = Gal3::Identity();

/* ************************************************************************* */
// Use GTSAM's Lie Group Test Macros
TEST(Gal3, LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(kIdentity_Lie, kIdentity_Lie);
  CHECK_LIE_GROUP_DERIVATIVES(kIdentity_Lie, kTestGal3_Lie1);
  CHECK_LIE_GROUP_DERIVATIVES(kTestGal3_Lie1, kIdentity_Lie);
  CHECK_LIE_GROUP_DERIVATIVES(kTestGal3_Lie1, kTestGal3_Lie2);
}

/* ************************************************************************* */
// Check derivatives of retract and localCoordinates
TEST(Gal3, ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(kIdentity_Lie, kIdentity_Lie);
  CHECK_CHART_DERIVATIVES(kIdentity_Lie, kTestGal3_Lie1);
  CHECK_CHART_DERIVATIVES(kTestGal3_Lie1, kIdentity_Lie);
  CHECK_CHART_DERIVATIVES(kTestGal3_Lie1, kTestGal3_Lie2);
}


/* ************************************************************************* */
TEST(Gal3, StaticConstructorsValue) {
    Gal3 g1 = Gal3::Create(kTestRot, kTestPos, kTestVel, kTestTime);
    EXPECT(assert_equal(kTestGal3, g1, kTol));

    Gal3 g2 = Gal3::FromPoseVelocityTime(kTestPose, kTestVel, kTestTime);
    EXPECT(assert_equal(kTestGal3, g2, kTol));
}

/* ************************************************************************* */
TEST(Gal3, ComponentAccessorsValue) {
    // Manifold
    EXPECT(assert_equal(kTestRot, kTestGal3.attitude(), kTol));
    EXPECT(assert_equal(kTestPos, kTestGal3.position(), kTol));
    EXPECT(assert_equal(kTestVel, kTestGal3.velocity(), kTol));
    EXPECT_DOUBLES_EQUAL(kTestTime, kTestGal3.time(), kTol);

    // Group
    EXPECT(assert_equal(kTestRot, kTestGal3.rotation(), kTol));
    EXPECT(assert_equal(kTestPos, kTestGal3.translation(), kTol));

    // Other
    EXPECT(assert_equal(kTestRot.matrix(), kTestGal3.R(), kTol));
    EXPECT(assert_equal(kTestPos, kTestGal3.r(), kTol));
    EXPECT(assert_equal(kTestVel, kTestGal3.v(), kTol));
    EXPECT_DOUBLES_EQUAL(kTestTime, kTestGal3.t(), kTol);
}

/* ************************************************************************* */
TEST(Gal3, MatrixConstructorValue) {
    Matrix5 M_known = kTestGal3.matrix();
    Gal3 g_from_matrix(M_known);
    EXPECT(assert_equal(kTestGal3, g_from_matrix, kTol));

    // Test invalid matrix construction (violating zero structure)
    Matrix5 M_invalid_zero = M_known;
    M_invalid_zero(4, 0) = 1e-5;
    try {
        Gal3 g_invalid(M_invalid_zero);
        FAIL("Constructor should have thrown for invalid matrix structure (zero violation).");
    } catch (const std::invalid_argument& e) {
        // Expected exception
    } catch (...) {
        FAIL("Constructor threw unexpected exception type for invalid matrix.");
    }

    // Test invalid matrix construction (violating M(3,3) == 1)
     Matrix5 M_invalid_diag = M_known;
     M_invalid_diag(3, 3) = 0.9;
     try {
        Gal3 g_invalid(M_invalid_diag);
        FAIL("Constructor should have thrown for invalid matrix structure (M(3,3) != 1).");
    } catch (const std::invalid_argument& e) {
        // Expected exception
    } catch (...) {
        FAIL("Constructor threw unexpected exception type for invalid matrix.");
    }
}

/* ************************************************************************* */
TEST(Gal3, Identity) {
    // Original hardcoded expected data (kept for functional equivalence)
    const Matrix3 expected_R_mat = Matrix3::Identity();
    const Vector3 expected_r_vec = Vector3::Zero();
    const Vector3 expected_v_vec = Vector3::Zero();
    const double expected_t_val = 0.0;

    Gal3 custom_ident = Gal3::Identity();

    // Original component checks (kept for functional equivalence)
    EXPECT(assert_equal(expected_R_mat, custom_ident.rotation().matrix(), kTol));
    EXPECT(assert_equal(Point3(expected_r_vec), custom_ident.translation(), kTol));
    EXPECT(assert_equal(expected_v_vec, custom_ident.velocity(), kTol));
    EXPECT_DOUBLES_EQUAL(expected_t_val, custom_ident.time(), kTol);

    // Original full object check (kept for functional equivalence)
    EXPECT(assert_equal(Gal3(Rot3(expected_R_mat), Point3(expected_r_vec), expected_v_vec, expected_t_val), custom_ident, kTol));
}

/* ************************************************************************* */
TEST(Gal3, HatVee) {
    // Test Case 1
    const Vector10 tau_vec_1 = (Vector10() <<
        -0.4665439537974297, -0.46391731412948783, -0.46638346398428376, // w
        -0.24728318780816563, 0.42470896104182426, 0.37654216726012074, // nu
        -0.9919387548344049, 0.41796965721894275, -0.08567669832855362, // rho
         -0.2703091399101363 // alpha
    ).finished();
    const Matrix5 expected_xi_matrix_1 = (Matrix5() <<
         0.0, 0.46638346398428376, -0.46391731412948783, -0.24728318780816563, -0.9919387548344049,
        -0.46638346398428376, 0.0, 0.4665439537974297, 0.42470896104182426, 0.41796965721894275,
         0.46391731412948783, -0.4665439537974297, 0.0, 0.37654216726012074, -0.08567669832855362,
         0.0, 0.0, 0.0, 0.0, -0.2703091399101363,
         0.0, 0.0, 0.0, 0.0, 0.0
    ).finished();

    Matrix5 custom_Xi_1 = Gal3::Hat(tau_vec_1);
    EXPECT(assert_equal(expected_xi_matrix_1, custom_Xi_1, kTol));

    Vector10 custom_tau_rec_1 = Gal3::Vee(expected_xi_matrix_1);
    EXPECT(assert_equal(tau_vec_1, custom_tau_rec_1, kTol));

    // Round trip check
    Vector10 custom_tau_rec_roundtrip_1 = Gal3::Vee(custom_Xi_1);
    EXPECT(assert_equal(tau_vec_1, custom_tau_rec_roundtrip_1, kTol));
}

/* ************************************************************************* */
TEST(Gal3, ExpmapZero) {
    Gal3::Jacobian aH;
    const Vector10 xi = Vector10::Zero();
    Gal3 custom_exp_1 = Gal3::Expmap(xi, aH);
    
    const Gal3 expected(Rot3(), Z_3x1, Z_3x1, 0);
    EXPECT(assert_equal(expected, custom_exp_1, kTol));

    // Check Jacobian
    std::function<Gal3(const Vector10&)> f = [](const Vector10& v){ return Gal3::Expmap(v); };
    Matrix expectedH = numericalDerivative11(f, xi);
    EXPECT(assert_equal(expectedH, aH, 1e-6));
}

/* ************************************************************************* */
TEST(Gal3, ExpmapBoost) {
    Gal3::Jacobian aH;
    Vector10 xi = Vector10::Zero();
    xi.segment<3>(3) = Vector3(1, 2, 3);
    Gal3 custom_exp_1 = Gal3::Expmap(xi, aH);
    
    const Gal3 expected(Rot3(), Z_3x1, Vector3(1, 2, 3), 0);
    EXPECT(assert_equal(expected, custom_exp_1, kTol));

    // Check Jacobian
    std::function<Gal3(const Vector10&)> f = [](const Vector10& v){ return Gal3::Expmap(v); };
    Matrix expectedH = numericalDerivative11(f, xi);
    EXPECT(assert_equal(expectedH, aH, 1e-6));
}

/* ************************************************************************* */
TEST(Gal3, Expmap) {
    const Vector10 xi = (Vector10() <<
    0.3267045270397072, -0.21318895514136532, -0.1810111529904679,  // w
    -0.24823309993679712, -0.3282613511681929, -0.3614305580886979,  // nu
    -0.6659680127970163, 0.0801034296770802, -0.005425197747099379,  // rho
    -0.11137002094819903 // alpha
    ).finished();
    const Matrix3 expected_R_mat_1 = (Matrix3() <<
         0.961491754653074, 0.14119138670272793, -0.23579354964696073,
        -0.20977429976081094, 0.9313179826319476, -0.297727322203087,
         0.17756223949368974, 0.33572579219851445, 0.925072885538575
    ).finished();
    const Point3 expected_r_vec_1(-0.637321673031978, 0.16116104619552254, -0.03248254605908951);
    const Velocity3 expected_v_vec_1(-0.22904001980446076, -0.23988916442951638, -0.4308710747620977);
    const double expected_t_val_1 = -0.11137002094819903;
    const Gal3 expected_exp_1(Rot3(expected_R_mat_1), expected_r_vec_1, expected_v_vec_1, expected_t_val_1);

    Gal3 custom_exp_1 = Gal3::Expmap(xi);
    EXPECT(assert_equal(expected_exp_1, custom_exp_1, kTol));

    // Check derivatives
    Gal3::Jacobian aH;
    Gal3::Expmap(xi, aH);
    std::function<Gal3(const Vector10&)> f = [](const Vector10& v){ return Gal3::Expmap(v); };
    Matrix expectedH = numericalDerivative11(f, xi);
    EXPECT(assert_equal(expectedH, aH, 1e-6));
}

/* ************************************************************************* */
TEST(Gal3, Logmap) {
    // Test case 1: Logmap(GroupElement)
    const Matrix3 input_R_mat_1 = (Matrix3() <<
        -0.8479395778141634, 0.26601628670932354, -0.4585126035145831,
         0.5159883846729487, 0.612401478276553, -0.5989327310201821,
         0.1214679351060988, -0.744445944720015, -0.6565407650184298
    ).finished();
    const Point3 input_r_vec_1(0.6584021866593519, -0.3393257406257678, -0.5420636579124554);
    const Velocity3 input_v_vec_1(0.1772802663861217, 0.3146080790621266, 0.7173535084539808);
    const double input_t_val_1 = -0.12088016100268817;
    const Gal3 custom_g_1(Rot3(input_R_mat_1), input_r_vec_1, input_v_vec_1, input_t_val_1);

    const Vector10 expected_log_tau_1 = (Vector10() <<
        -0.6312616056853186, -2.516056355068653, 1.0844223965979727,  // w
        1.122213550821757, -0.21828427331226408, 0.03100839182220047, // nu
        -0.6366686897004876, -0.2565186503803428, -1.1108185946230884, // rho
        -0.12088016100268817 // alpha
    ).finished();

    Vector10 custom_log_tau_1 = Gal3::Logmap(custom_g_1);
    // Note: Logmap can have higher errors near singularities
    EXPECT(assert_equal(expected_log_tau_1, custom_log_tau_1, kTol * 1e3));

    // Test Log(Exp(tau)) round trip (using data from Expmap test)
    const Vector10 tau_vec_orig_rt1 = (Vector10() <<
        -0.6659680127970163, 0.0801034296770802, -0.005425197747099379,
        -0.24823309993679712, -0.3282613511681929, -0.3614305580886979,
         0.3267045270397072, -0.21318895514136532, -0.1810111529904679,
        -0.11137002094819903
    ).finished();
    Gal3 g_exp_rt1 = Gal3::Expmap(tau_vec_orig_rt1);
    Vector10 tau_log_exp_rt1 = Gal3::Logmap(g_exp_rt1);
    EXPECT(assert_equal(tau_vec_orig_rt1, tau_log_exp_rt1, kTol * 10));

    // Test Exp(Log(g)) round trip (using data from first Logmap test)
    Gal3 custom_g_orig_rt2 = custom_g_1;
    Vector10 tau_log_rt2 = Gal3::Logmap(custom_g_orig_rt2);
    Gal3 g_exp_log_rt2 = Gal3::Expmap(tau_log_rt2);
    EXPECT(assert_equal(custom_g_orig_rt2, g_exp_log_rt2, kTol * 10));
}

/* ************************************************************************* */
TEST(Gal3, Compose) {
    // Test Case 1
    const Matrix3 g1_R_mat_1 = (Matrix3() <<
        -0.5427153955878299, 0.8391431164114453, 0.03603927817303032,
         0.8040805715986894, 0.5314810596281534, -0.2664250694549225,
        -0.24272295682417533, -0.11561450357036887, -0.963181630220753
    ).finished();
    const Point3 g1_r_vec_1(0.9978490360071179, -0.5634861893781862, 0.025864788808796835);
    const Velocity3 g1_v_vec_1(0.04857438013356852, -0.012834026018545996, 0.945550047767139);
    const double g1_t_val_1 = -0.41496643117394594;
    const Gal3 c1_1(Rot3(g1_R_mat_1), g1_r_vec_1, g1_v_vec_1, g1_t_val_1);

    const Matrix3 g2_R_mat_1 = (Matrix3() <<
        -0.3264538540162394, 0.24276278793202133, -0.9135064914894779,
         0.9430076454867458, 0.1496317101600385, -0.2972321178273404,
         0.06453264097716288, -0.9584761760784951, -0.27777501352435885
    ).finished();
    const Point3 g2_r_vec_1(-0.1995427558196442, 0.7830589040103644, -0.433370507989717);
    const Velocity3 g2_v_vec_1(0.8986541507293722, 0.051990700444202176, -0.8278883042875157);
    const double g2_t_val_1 = -0.6155723080111539;
    const Gal3 c2_1(Rot3(g2_R_mat_1), g2_r_vec_1, g2_v_vec_1, g2_t_val_1);

    const Matrix3 expected_R_mat_1 = (Matrix3() <<
         0.9708156167565788, -0.04073147244077803, 0.23634294026763253,
         0.22150238776832248, 0.5300893429357028, -0.8184998355032984,
        -0.09194417042137741, 0.8469629482207595, 0.5236411308011658
    ).finished();
    const Point3 expected_r_vec_1(1.7177230471349891, -0.18439262777314758, -0.18087448280827323);
    const Velocity3 expected_v_vec_1(-0.4253479212754224, 0.9579585887030762, 1.5188219826819997);
    const double expected_t_val_1 = -1.0305387391850998;
    const Gal3 expected_comp_1(Rot3(expected_R_mat_1), expected_r_vec_1, expected_v_vec_1, expected_t_val_1);

    Gal3 custom_comp_1 = c1_1 * c2_1; // Or c1_1.compose(c2_1)
    EXPECT(assert_equal(expected_comp_1, custom_comp_1, kTol));
}

/* ************************************************************************* */
TEST(Gal3, Inverse) {
    Gal3 expected_identity = Gal3::Identity();

    // Test Case 1
    const Matrix3 g_R_mat_1 = (Matrix3() <<
         0.6680516673568877, 0.2740542884848495, -0.6918101016209183,
         0.6729369985913887, -0.6193062871756463, 0.4044941514923666,
        -0.31758898858193396, -0.7357676057205693, -0.5981498680963873
    ).finished();
    const Point3 g_r_vec_1(0.06321286832132045, -0.9214393132931736, -0.12472480681013542);
    const Velocity3 g_v_vec_1(0.4770686298036335, 0.2799576331302327, -0.29190264050471715);
    const double g_t_val_1 = 0.3757227805330059;
    const Gal3 custom_g_1(Rot3(g_R_mat_1), g_r_vec_1, g_v_vec_1, g_t_val_1);

    const Matrix3 expected_inv_R_mat_1 = (Matrix3() <<
         0.6680516673568877, 0.6729369985913887, -0.31758898858193396,
         0.2740542884848495, -0.6193062871756463, -0.7357676057205693,
        -0.6918101016209183, 0.4044941514923666, -0.5981498680963873
    ).finished();
    const Point3 expected_inv_r_vec_1(0.7635904739613719, -0.6150700906051861, 0.32598918251792036);
    const Velocity3 expected_inv_v_vec_1(-0.5998054073176801, -0.17213568846657853, 0.042198146082895516);
    const double expected_inv_t_val_1 = -0.3757227805330059;
    const Gal3 expected_inv_1(Rot3(expected_inv_R_mat_1), expected_inv_r_vec_1, expected_inv_v_vec_1, expected_inv_t_val_1);

    Gal3 custom_inv_1 = custom_g_1.inverse();
    EXPECT(assert_equal(expected_inv_1, custom_inv_1, kTol));

    // Check g * g.inverse() == Identity
    Gal3 ident_c_1 = custom_g_1 * custom_inv_1;
    EXPECT(assert_equal(expected_identity, ident_c_1, kTol));
}

/* ************************************************************************* */
TEST(Gal3, Between) {
     // Test Case 1
    const Matrix3 g1_R_mat_1 = (Matrix3() <<
        -0.2577418495238488, 0.7767168385303765, 0.5747000015202739,
        -0.6694062876067332, -0.572463082520484, 0.47347781496466923,
         0.6967527259484512, -0.26267274676776586, 0.6674868290752107
    ).finished();
    const Point3 g1_r_vec_1(0.680375434309419, -0.21123414636181392, 0.5661984475172117);
    const Velocity3 g1_v_vec_1(0.536459189623808, -0.44445057839362445, 0.10793991159086103);
    const double g1_t_val_1 = -0.0452058962756795;
    const Gal3 c1_1(Rot3(g1_R_mat_1), g1_r_vec_1, g1_v_vec_1, g1_t_val_1);

    const Matrix3 g2_R_mat_1 = (Matrix3() <<
         0.1981112115076329, -0.12884463237051902, 0.9716743325745948,
         0.9776658305457298, -0.04497580389201028, -0.20529661674659028,
         0.0701532013404006, 0.9906443548385938, 0.11705678352030657
    ).finished();
    const Point3 g2_r_vec_1(0.9044594503494257, 0.8323901360074013, 0.2714234559198019);
    const Velocity3 g2_v_vec_1(-0.5142264587405261, -0.7255368464279626, 0.6083535084539808);
    const double g2_t_val_1 = -0.6866418214918308;
    const Gal3 c2_1(Rot3(g2_R_mat_1), g2_r_vec_1, g2_v_vec_1, g2_t_val_1);

    // Expected: c1.inverse() * c2
    const Matrix3 expected_R_mat_1 = (Matrix3() <<
        -0.6566377699430239, 0.753549894443112, -0.0314546605295386,
        -0.4242286152401611, -0.3345440819370167, 0.8414929228771536,
         0.6235839326792096, 0.5659000033801861, 0.5393517081447288
    ).finished();
    const Point3 expected_r_vec_1(-0.8113603292280276, 0.06632931940009146, 0.535144598419476);
    const Velocity3 expected_v_vec_1(0.8076311151757332, -0.7866187376416414, -0.4028976707214998);
    const double expected_t_val_1 = -0.6414359252161513;
    const Gal3 expected_btw_1(Rot3(expected_R_mat_1), expected_r_vec_1, expected_v_vec_1, expected_t_val_1);

    Gal3 custom_btw_1 = c1_1.between(c2_1);
    EXPECT(assert_equal(expected_btw_1, custom_btw_1, kTol));
}

/* ************************************************************************* */
TEST(Gal3, MatrixComponents) {
    // Test Case 1
    const Matrix3 source_R_mat_1 = (Matrix3() <<
         0.43788117516687186, -0.12083239518241493, -0.8908757538001356,
         0.4981128609611659, 0.8575347951217139, 0.12852102124027118,
         0.7484274541861499, -0.5000336063006573, 0.43568651389548174
    ).finished();
    const Point3 source_r_vec_1(0.3684370505476542, 0.8219440615838134, -0.03501868668711683);
    const Velocity3 source_v_vec_1(0.7621243390078305, 0.282161192634218, -0.13609316346053646);
    const double source_t_val_1 = 0.23919296788014144;
    const Gal3 c1(Rot3(source_R_mat_1), source_r_vec_1, source_v_vec_1, source_t_val_1);

    Matrix5 expected_Mc;
    expected_Mc << source_R_mat_1, source_v_vec_1, source_r_vec_1,
                   Vector3::Zero().transpose(), 1.0, source_t_val_1,
                   Vector4::Zero().transpose(), 1.0;

    Matrix5 Mc = c1.matrix();
    EXPECT(assert_equal(expected_Mc, Mc, kTol));
}

/* ************************************************************************* */
TEST(Gal3, Associativity) {
    // Test Case 1
    const Vector10 tau1_1 = (Vector10() <<
         0.14491869060866264, -0.21431172810692092, -0.4956042914756127,
        -0.13411549788475238, 0.44534636811395767, -0.33281648500962074,
        -0.012095339359589743, -0.20104157502639808, 0.08197507996416392,
        -0.006285789459736368
    ).finished();
    const Vector10 tau2_1 = (Vector10() <<
         0.29551108535517384, 0.2938672197508704, 0.0788811297200055,
        -0.07107463852205165, 0.22379088709954872, -0.26508231911443875,
        -0.11625916165500054, 0.04661407377867886, 0.1788274027858556,
        -0.015243024791797033
    ).finished();
     const Vector10 tau3_1 = (Vector10() <<
         0.37646834040234084, 0.3782542871960659, -0.6525520272351378,
         0.005426723127791683, 0.09951505587485733, 0.3813252061414707,
        -0.09289643299325814, -0.0017149201262141199, -0.08507973168896384,
        -0.1109049754985476
    ).finished();

    Gal3 g1_1 = Gal3::Expmap(tau1_1);
    Gal3 g2_1 = Gal3::Expmap(tau2_1);
    Gal3 g3_1 = Gal3::Expmap(tau3_1);

    Gal3 left_assoc_1 = (g1_1 * g2_1) * g3_1;
    Gal3 right_assoc_1 = g1_1 * (g2_1 * g3_1);

    // Use slightly larger tolerance for composed operations
    EXPECT(assert_equal(left_assoc_1, right_assoc_1, kTol * 10));
}

/* ************************************************************************* */
TEST(Gal3, IdentityProperties) {
    Gal3 custom_identity = Gal3::Identity();

    // Test data
    const Matrix3 g_R_mat_1 = (Matrix3() <<
        -0.5204974727334908, 0.7067813015326174, 0.4791060140322894,
         0.773189425449982, 0.15205374379417114, 0.6156766776243058,
         0.3622989004266723, 0.6908978584436601, -0.6256194178153267
    ).finished();
    const Point3 g_r_vec_1(-0.8716573584227159, -0.9599539022706234, -0.08459652545144625);
    const Velocity3 g_v_vec_1(0.7018395735425127, -0.4666685012479632, 0.07952068144433233);
    const double g_t_val_1 = -0.24958604725524136;
    const Gal3 custom_g_1(Rot3(g_R_mat_1), g_r_vec_1, g_v_vec_1, g_t_val_1);

    // g * g.inverse() == identity
    Gal3 g_inv_1 = custom_g_1.inverse();
    Gal3 g_times_inv_1 = custom_g_1 * g_inv_1;
    EXPECT(assert_equal(custom_identity, g_times_inv_1, kTol * 10)); // Use slightly larger tol for round trip

    // identity * g == g
    Gal3 id_times_g_1 = custom_identity * custom_g_1;
    EXPECT(assert_equal(custom_g_1, id_times_g_1, kTol));

    // g * identity == g
    Gal3 g_times_id_1 = custom_g_1 * custom_identity;
    EXPECT(assert_equal(custom_g_1, g_times_id_1, kTol));
}

/* ************************************************************************* */
TEST(Gal3, Adjoint) {
    // Test data
    const Matrix3 g_R_mat_1 = (Matrix3() <<
        -0.2577418495238488, 0.7767168385303765, 0.5747000015202739,
        -0.6694062876067332, -0.572463082520484, 0.47347781496466923,
         0.6967527259484512, -0.26267274676776586, 0.6674868290752107
    ).finished();
    const Point3 g_r_vec_1(0.680375434309419, -0.21123414636181392, 0.5661984475172117);
    const Velocity3 g_v_vec_1(0.536459189623808, -0.44445057839362445, 0.10793991159086103);
    const double g_t_val_1 = -0.0452058962756795;
    const Gal3 test_g(Rot3(g_R_mat_1), g_r_vec_1, g_v_vec_1, g_t_val_1);

    // Call the generic AdjointMap from the base class
    Gal3::Jacobian expected_adj_matrix =
        static_cast<const MatrixLieGroup<Gal3, 10, 5>*>(&test_g)->AdjointMap();

    Gal3::Jacobian computed_adj = test_g.AdjointMap();
    EXPECT(assert_equal(expected_adj_matrix, computed_adj, kTol * 10)); // Slightly larger tolerance

    // Test tangent vector for adjoint action
    Vector10 xi = (Vector10() <<
        -0.28583171387804845, -0.7221038902918132, -0.09516831208249353,
        -0.13619637934919504, -0.05432836001072756, 0.1629798883384306,
        -0.20194877118636279, -0.18928645162443278, 0.07312685929426145,
        -0.042327821984942095
    ).finished();

    // Test the adjoint property: Log(g * Exp(tau) * g^-1) = Ad(g) * tau
    Gal3 exp_tau = Gal3::Expmap(xi);
    Gal3 g_exp_tau_ginv = test_g * exp_tau * test_g.inverse();
    Vector10 log_result = Gal3::Logmap(g_exp_tau_ginv);

    // Compare Log(g*Exp(tau)*g^-1) with Ad(g)*tau
    Vector10 expected_adj_xi = test_g.Adjoint(xi);
    EXPECT(assert_equal(expected_adj_xi, log_result, kTol * 10)); // Use larger tolerance for Exp/Log round trip
}

/* ************************************************************************* */
// compose and inverse derivatives are good once AdjointMap is good. Lie.h

/* ************************************************************************* */
TEST(Gal3, Jacobian_Logmap) {
    const double jac_tol = 1e-5; // Tolerance for Jacobian checks

    // Test Case 1
    const Matrix3 R1_mat = (Matrix3() <<
        -0.5204974727334908, 0.7067813015326174, 0.4791060140322894,
         0.773189425449982,  0.15205374379417114, 0.6156766776243058,
         0.3622989004266723,  0.6908978584436601, -0.6256194178153267
    ).finished();
    const Point3 r1_vec(-0.8716573584227159, -0.9599539022706234, -0.08459652545144625);
    const Velocity3 v1_vec(0.7018395735425127, -0.4666685012479632, 0.07952068144433233);
    const double t1_val = -0.24958604725524136;
    const Gal3 g1(Rot3(R1_mat), r1_vec, v1_vec, t1_val);

    const Vector10 expected_log_tau1 = (Vector10() <<
        1.4969846781977756,  2.324590726540746,  1.321595100333433,
        -0.34870126525214656, -0.4153032457886797, 1.1791315551702946,
        -1.2604528322799349, -0.3898722924179116, -0.6402392791385879,
        -0.24958604725524136
    ).finished();

    Gal3::Jacobian Hg1_gtsam;
    Vector10 log1_gtsam = Gal3::Logmap(g1, Hg1_gtsam);
    EXPECT(assert_equal(expected_log_tau1, log1_gtsam, kTol));

    // Verify Jacobian against numerical derivative
    std::function<Vector10(const Gal3&)> logmap_func1 =
        [](const Gal3& g_in) { return Gal3::Logmap(g_in); };
    Matrix H_numerical1 = numericalDerivative11<Vector10, Gal3>(logmap_func1, g1, jac_tol);
    EXPECT(assert_equal(H_numerical1, Hg1_gtsam, jac_tol));

    // Test Case 2
    const Matrix3 R2_mat = (Matrix3() <<
         0.6680516673568877, 0.2740542884848495, -0.6918101016209183,
         0.6729369985913887,-0.6193062871756463,  0.4044941514923666,
        -0.31758898858193396,-0.7357676057205693, -0.5981498680963873
    ).finished();
    const Point3 r2_vec(0.06321286832132045, -0.9214393132931736, -0.12472480681013542);
    const Velocity3 v2_vec(0.4770686298036335, 0.2799576331302327, -0.29190264050471715);
    const double t2_val = 0.3757227805330059;
    const Gal3 g2(Rot3(R2_mat), r2_vec, v2_vec, t2_val);

    const Vector10 expected_log_tau2 = (Vector10() <<
        -2.215366977027571,  -0.72705858167113,   0.7749725693135466,
        0.5179342682694047,  0.3616924279678234, -0.0984011207117694,
        -0.5687147057967125, -0.3805406510759017, -1.063343079044753,
         0.3757227805330059
    ).finished();

    Gal3::Jacobian Hg2_gtsam;
    Vector10 log2_gtsam = Gal3::Logmap(g2, Hg2_gtsam);
    EXPECT(assert_equal(expected_log_tau2, log2_gtsam, kTol));

    // Verify Jacobian against numerical derivative
    std::function<Vector10(const Gal3&)> logmap_func2 =
        [](const Gal3& g_in) { return Gal3::Logmap(g_in); };
    Matrix H_numerical2 = numericalDerivative11<Vector10, Gal3>(logmap_func2, g2, jac_tol);
    EXPECT(assert_equal(H_numerical2, Hg2_gtsam, jac_tol));
}

/* ************************************************************************* */
TEST(Gal3, Jacobian_Expmap) {
    // Test data
    Vector10 xi = (Vector10() <<
        0.3779854061644677, -0.09638288751073966, -0.2917351793587256,
        0.06314401798217141, -0.07707725418679535, -0.26078036994807674,
        0.1644755309744135, -0.212580759622502, 0.027598787765563664,
        -0.49338105141355293
    ).finished();

    Gal3::Jacobian aH;
    Gal3::Expmap(xi, aH);

    // Check derivatives
    std::function<Gal3(const Vector10&)> f = [](const Vector10& v){ return Gal3::Expmap(v); };
    Matrix expectedH = numericalDerivative11(f, xi);
    EXPECT(assert_equal(expectedH, aH, 1e-6));
}

/* ************************************************************************* */
// Test Between Jacobian against numerical derivatives
TEST(Gal3, Jacobian_Between) {
    // Test data
    Matrix3 g1_R_mat = (Matrix3() <<
         0.1981112115076329, -0.12884463237051902, 0.9716743325745948,
         0.9776658305457298, -0.04497580389201028, -0.20529661674659028,
         0.0701532013404006, 0.9906443548385938, 0.11705678352030657
    ).finished();
    Point3 g1_r_vec(0.9044594503494257, 0.8323901360074013, 0.2714234559198019);
    Velocity3 g1_v_vec(-0.5142264587405261, -0.7255368464279626, 0.6083535084539808);
    double g1_t_val = -0.6866418214918308;
    Gal3 g1(Rot3(g1_R_mat), g1_r_vec, g1_v_vec, g1_t_val);

    Matrix3 g2_R_mat = (Matrix3() <<
        -0.5427153955878299, 0.8391431164114453, 0.03603927817303032,
         0.8040805715986894, 0.5314810596281534, -0.2664250694549225,
        -0.24272295682417533, -0.11561450357036887, -0.963181630220753
    ).finished();
    Point3 g2_r_vec(0.9978490360071179, -0.5634861893781862, 0.025864788808796835);
    Velocity3 g2_v_vec(0.04857438013356852, -0.012834026018545996, 0.945550047767139);
    double g2_t_val = -0.41496643117394594;
    Gal3 g2(Rot3(g2_R_mat), g2_r_vec, g2_v_vec, g2_t_val);

    Gal3::Jacobian H1_analytical, H2_analytical;
    g1.between(g2, H1_analytical, H2_analytical);

    std::function<Gal3(const Gal3&, const Gal3&)> between_func =
        [](const Gal3& g1_in, const Gal3& g2_in) { return g1_in.between(g2_in); };

    // Verify H1
    Matrix H1_numerical = numericalDerivative21(between_func, g1, g2, 1e-6);
    EXPECT(assert_equal(H1_numerical, H1_analytical, 1e-5)); // Tolerance for numerical comparison

    // Verify H2
    Matrix H2_numerical = numericalDerivative22(between_func, g1, g2, 1e-6);
    EXPECT(assert_equal(H2_numerical, H2_analytical, 1e-5)); // Tolerance for numerical comparison
}


/* ************************************************************************* */
TEST(Gal3, Jacobian_AdjointMap) {
    // Test data
    Matrix3 g1_R_mat = (Matrix3() <<
         0.1981112115076329, -0.12884463237051902, 0.9716743325745948,
         0.9776658305457298, -0.04497580389201028, -0.20529661674659028,
         0.0701532013404006, 0.9906443548385938, 0.11705678352030657
    ).finished();
    Point3 g1_r_vec(0.9044594503494257, 0.8323901360074013, 0.2714234559198019);
    Velocity3 g1_v_vec(-0.5142264587405261, -0.7255368464279626, 0.6083535084539808);
    double g1_t_val = -0.6866418214918308;
    Gal3 g = Gal3(Rot3(g1_R_mat), g1_r_vec, g1_v_vec, g1_t_val);

    Vector10 xi = (Vector10() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.1, 0.15, -0.25, 0.9).finished();

    // Analytical Adjoint Map
    Gal3::Jacobian Ad_analytical = g.AdjointMap();

    // Numerical derivative of Adjoint action Ad_g(xi) w.r.t xi should equal Adjoint Map
    std::function<Vector10(const Gal3&, const Vector10&)> adjoint_action =
        [](const Gal3& g_in, const Vector10& xi_in) {
            return g_in.Adjoint(xi_in);
        };
    Matrix H_xi_numerical = numericalDerivative22(adjoint_action, g, xi);
    EXPECT(assert_equal(Ad_analytical, H_xi_numerical, 1e-7)); // Tolerance for numerical diff

    // Verify derivative of Adjoint action Ad_g(xi) w.r.t g
    Gal3::Jacobian H_g_analytical;
    g.Adjoint(xi, H_g_analytical); // Calculate analytical H_g

    // Need wrapper that only returns value for numericalDerivative21
    std::function<Vector10(const Gal3&, const Vector10&)> adjoint_action_wrt_g_val =
      [](const Gal3& g_in, const Vector10& xi_in) {
        return g_in.Adjoint(xi_in); // Return only value
      };
    Matrix H_g_numerical = numericalDerivative21(adjoint_action_wrt_g_val, g, xi);
    EXPECT(assert_equal(H_g_analytical, H_g_numerical, 1e-7));
}

/* ************************************************************************* */
TEST(Gal3, Act) {
    const double kTol = 1e-9;      // Tolerance for value checks
    const double jac_tol = 1e-6;   // Tolerance for Jacobian checks

    // Test Case 1 Data
    const Matrix3 R1_mat = (Matrix3() <<
       -0.2577418495238488,  0.7767168385303765,  0.5747000015202739,
       -0.6694062876067332, -0.572463082520484,   0.47347781496466923,
        0.6967527259484512, -0.26267274676776586, 0.6674868290752107
    ).finished();
    const Point3 r1_vec(0.680375434309419, -0.21123414636181392, 0.5661984475172117);
    const Velocity3 v1_vec(0.536459189623808, -0.44445057839362445, 0.10793991159086103);
    const double t1_val = -0.0452058962756795;
    const Gal3 g1(Rot3(R1_mat), r1_vec, v1_vec, t1_val);
    const Point3 point_in1(4.967141530112327, -1.3826430117118464, 6.476885381006925);
    const double time_in1 = 0.0;
    const Event e_in1(time_in1, point_in1);
    const Point3 expected_p_out1 = g1.rotation().rotate(point_in1) + g1.velocity() * time_in1 + g1.translation();
    const double expected_t_out1 = time_in1 + t1_val;

    Matrix H1g_gtsam = Matrix::Zero(4, 10);
    Matrix H1e_gtsam = Matrix::Zero(4, 4);
    Event e_out1_gtsam = g1.act(e_in1, H1g_gtsam, H1e_gtsam);

    EXPECT(assert_equal(expected_p_out1, e_out1_gtsam.location(), kTol));
    EXPECT_DOUBLES_EQUAL(expected_t_out1, e_out1_gtsam.time(), kTol);

    // Verify H1g: Derivative of output Event wrt Gal3 g1
    std::function<Point3(const Gal3&, const Event&)> act_func1_loc_wrt_g =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).location(); };
    Matrix H1g_loc_numerical = numericalDerivative21<Point3, Gal3, Event>(act_func1_loc_wrt_g, g1, e_in1, jac_tol);
    EXPECT(assert_equal(H1g_loc_numerical, H1g_gtsam.block<3, 10>(1, 0), jac_tol));

    std::function<double(const Gal3&, const Event&)> act_func1_time_wrt_g =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).time(); };
    Matrix H1g_time_numerical = numericalDerivative21<double, Gal3, Event>(act_func1_time_wrt_g, g1, e_in1, jac_tol);
    EXPECT(assert_equal(H1g_time_numerical, H1g_gtsam.row(0), jac_tol));

    // Verify H1e: Derivative of output Event wrt Event e1
    std::function<Point3(const Gal3&, const Event&)> act_func1_loc_wrt_e =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).location(); };
    Matrix H1e_loc_numerical = numericalDerivative22<Point3, Gal3, Event>(act_func1_loc_wrt_e, g1, e_in1, jac_tol);
    EXPECT(assert_equal(H1e_loc_numerical, H1e_gtsam.block<3, 4>(1, 0), jac_tol));

    std::function<double(const Gal3&, const Event&)> act_func1_time_wrt_e =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).time(); };
    Matrix H1e_time_numerical = numericalDerivative22<double, Gal3, Event>(act_func1_time_wrt_e, g1, e_in1, jac_tol);
    EXPECT(assert_equal(H1e_time_numerical, H1e_gtsam.row(0), jac_tol));

    // Test Case 2 Data
    const Matrix3 R2_mat = (Matrix3() <<
        0.1981112115076329, -0.12884463237051902, 0.9716743325745948,
        0.9776658305457298, -0.04497580389201028, -0.20529661674659028,
        0.0701532013404006,  0.9906443548385938,  0.11705678352030657
    ).finished();
    const Point3 r2_vec(0.9044594503494257, 0.8323901360074013, 0.2714234559198019);
    const Velocity3 v2_vec(-0.5142264587405261, -0.7255368464279626, 0.6083535084539808);
    const double t2_val = -0.6866418214918308;
    const Gal3 g2(Rot3(R2_mat), r2_vec, v2_vec, t2_val);
    const Point3 point_in2(15.230298564080254, -2.3415337472333597, -2.3413695694918055);
    const double time_in2 = 0.0;
    const Event e_in2(time_in2, point_in2);
    const Point3 expected_p_out2 = g2.rotation().rotate(point_in2) + g2.velocity() * time_in2 + g2.translation();
    const double expected_t_out2 = time_in2 + t2_val;

    Matrix H2g_gtsam = Matrix::Zero(4, 10);
    Matrix H2e_gtsam = Matrix::Zero(4, 4);
    Event e_out2_gtsam = g2.act(e_in2, H2g_gtsam, H2e_gtsam);

    EXPECT(assert_equal(expected_p_out2, e_out2_gtsam.location(), kTol));
    EXPECT_DOUBLES_EQUAL(expected_t_out2, e_out2_gtsam.time(), kTol);

    // Verify H2g: Derivative of output Event wrt Gal3 g2
    std::function<Point3(const Gal3&, const Event&)> act_func2_loc_wrt_g =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).location(); };
    Matrix H2g_loc_numerical = numericalDerivative21(act_func2_loc_wrt_g, g2, e_in2, jac_tol);
    EXPECT(assert_equal(H2g_loc_numerical, H2g_gtsam.block<3, 10>(1, 0), jac_tol));

    std::function<double(const Gal3&, const Event&)> act_func2_time_wrt_g =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).time(); };
    Matrix H2g_time_numerical = numericalDerivative21(act_func2_time_wrt_g, g2, e_in2, jac_tol);
    EXPECT(assert_equal(H2g_time_numerical, H2g_gtsam.row(0), jac_tol));

    // Verify H2e: Derivative of output Event wrt Event e2
    std::function<Point3(const Gal3&, const Event&)> act_func2_loc_wrt_e =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).location(); };
    Matrix H2e_loc_numerical = numericalDerivative22(act_func2_loc_wrt_e, g2, e_in2, jac_tol);
    EXPECT(assert_equal(H2e_loc_numerical, H2e_gtsam.block<3, 4>(1, 0), jac_tol));

    std::function<double(const Gal3&, const Event&)> act_func2_time_wrt_e =
        [](const Gal3& g_in, const Event& e_in) { return g_in.act(e_in).time(); };
    Matrix H2e_time_numerical = numericalDerivative22(act_func2_time_wrt_e, g2, e_in2, jac_tol);
    EXPECT(assert_equal(H2e_time_numerical, H2e_gtsam.row(0), jac_tol));
}

/* ************************************************************************* */
TEST(Gal3, Interpolate) {
    const double interp_tol = 1e-6; // Tolerance for interpolation value comparison

    // Test data
    const Matrix3 R1_mat = (Matrix3() <<
       -0.5427153955878299, 0.8391431164114453,  0.03603927817303032,
        0.8040805715986894, 0.5314810596281534, -0.2664250694549225,
       -0.24272295682417533,-0.11561450357036887,-0.963181630220753
    ).finished();
    const Point3 r1_vec(0.9978490360071179, -0.5634861893781862, 0.025864788808796835);
    const Velocity3 v1_vec(0.04857438013356852, -0.012834026018545996, 0.945550047767139);
    const double t1_val = -0.41496643117394594;
    const Gal3 g1(Rot3(R1_mat), r1_vec, v1_vec, t1_val);

    const Matrix3 R2_mat = (Matrix3() <<
       -0.3264538540162394, 0.24276278793202133, -0.9135064914894779,
        0.9430076454867458, 0.1496317101600385,  -0.2972321178273404,
        0.06453264097716288,-0.9584761760784951,  -0.27777501352435885
    ).finished();
    const Point3 r2_vec(-0.1995427558196442, 0.7830589040103644, -0.433370507989717);
    const Velocity3 v2_vec(0.8986541507293722, 0.051990700444202176, -0.8278883042875157);
    const double t2_val = -0.6155723080111539;
    const Gal3 g2(Rot3(R2_mat), r2_vec, v2_vec, t2_val);

    // Expected results for different alphas
    const Gal3 expected_alpha0_00 = g1;
    const Gal3 expected_alpha0_25(Rot3(-0.5305293740379828,  0.8049951577356123, -0.26555861745588893,
                                        0.8357380608208966,  0.444360967494815,  -0.32262241748272774,
                                       -0.14170559967126867, -0.39309811318459514,-0.9085116380281089),
                                  Point3(0.7319725410358775, -0.24509083842031143, -0.20526665070473993),
                                  Velocity3(0.447233815335834, 0.08156238532481315, 0.6212732810121263),
                                  -0.46511790038324796);
    const Gal3 expected_alpha0_50(Rot3(-0.4871812472793253,  0.6852678695634308, -0.5413523614461815,
                                        0.8717937932763143,  0.34522257149684665,-0.34755856791913387,
                                       -0.051283665082120816,-0.6412716453057169, -0.7655982383878919),
                                  Point3(0.4275876127256323, 0.0806397132393819, -0.3622648631703127),
                                  Velocity3(0.7397753319939113, 0.12027591889377548, 0.19009330402180313),
                                  -0.5152693695925499);
    const Gal3 expected_alpha0_75(Rot3(-0.416880477991759,   0.49158776471130083,-0.7645600935541361,
                                        0.9087464582621175,  0.24369303223618222,-0.338812013712018,
                                        0.019762127046961175,-0.8360353913714909, -0.5483195078682666),
                                  Point3(0.10860773480095642, 0.42140693978775756, -0.4380637787537704),
                                  Velocity3(0.895925123398753, 0.10738792759782673, -0.3083508669060544),
                                  -0.5654208388018519);
    const Gal3 expected_alpha1_00 = g2;

    // Compare values (requires Gal3::interpolate implementation)
    EXPECT(assert_equal(expected_alpha0_00, interpolate(g1, g2, 0.0), interp_tol));
    EXPECT(assert_equal(expected_alpha0_25, interpolate(g1, g2, 0.25), interp_tol));
    EXPECT(assert_equal(expected_alpha0_50, interpolate(g1, g2, 0.5), interp_tol));
    EXPECT(assert_equal(expected_alpha0_75, interpolate(g1, g2, 0.75), interp_tol));
    EXPECT(assert_equal(expected_alpha1_00, interpolate(g1, g2, 1.0), interp_tol));
}

/* ************************************************************************* */
TEST(Gal3, Jacobian_StaticConstructors) {
    // Verify analytical Jacobians of constructors against numerical derivatives.
    // Assumes Gal3::Create and Gal3::FromPoseVelocityTime implement these Jacobians.
    const double jac_tol = 1e-7; // Tolerance for Jacobian checks

    // Test Gal3::Create
    Matrix H1_ana, H2_ana, H3_ana, H4_ana;
    Gal3::Create(kTestRot, kTestPos, kTestVel, kTestTime, H1_ana, H2_ana, H3_ana, H4_ana);

    std::function<Gal3(const Rot3&, const Point3&, const Velocity3&, const double&)> create_func =
        [](const Rot3& R, const Point3& r, const Velocity3& v, const double& t){
            return Gal3::Create(R, r, v, t); // Call without Jacobians for numerical diff
        };

    const double& time_ref = kTestTime; // Needed for lambda capture if using C++11/14
    Matrix H1_num = numericalDerivative41(create_func, kTestRot, kTestPos, kTestVel, time_ref, jac_tol);
    Matrix H2_num = numericalDerivative42(create_func, kTestRot, kTestPos, kTestVel, time_ref, jac_tol);
    Matrix H3_num = numericalDerivative43(create_func, kTestRot, kTestPos, kTestVel, time_ref, jac_tol);
    Matrix H4_num = numericalDerivative44(create_func, kTestRot, kTestPos, kTestVel, time_ref, jac_tol);

    EXPECT(assert_equal(H1_num, H1_ana, jac_tol));
    EXPECT(assert_equal(H2_num, H2_ana, jac_tol));
    EXPECT(assert_equal(H3_num, H3_ana, jac_tol));
    EXPECT(assert_equal(H4_num, H4_ana, jac_tol));

    // Test Gal3::FromPoseVelocityTime
    Matrix Hpv1_ana, Hpv2_ana, Hpv3_ana;
    Gal3::FromPoseVelocityTime(kTestPose, kTestVel, kTestTime, Hpv1_ana, Hpv2_ana, Hpv3_ana);

    std::function<Gal3(const Pose3&, const Velocity3&, const double&)> fromPoseVelTime_func =
        [](const Pose3& pose, const Velocity3& v, const double& t){
            return Gal3::FromPoseVelocityTime(pose, v, t); // Call without Jacobians
        };

    Matrix Hpv1_num = numericalDerivative31(fromPoseVelTime_func, kTestPose, kTestVel, time_ref, jac_tol);
    Matrix Hpv2_num = numericalDerivative32(fromPoseVelTime_func, kTestPose, kTestVel, time_ref, jac_tol);
    Matrix Hpv3_num = numericalDerivative33(fromPoseVelTime_func, kTestPose, kTestVel, time_ref, jac_tol);

    EXPECT(assert_equal(Hpv1_num, Hpv1_ana, jac_tol));
    EXPECT(assert_equal(Hpv2_num, Hpv2_ana, jac_tol));
    EXPECT(assert_equal(Hpv3_num, Hpv3_ana, jac_tol));
}

/* ************************************************************************* */
TEST(Gal3, Jacobian_Accessors) {
    // Verify analytical Jacobians of accessors against numerical derivatives.
    // Assumes accessors implement these Jacobians.
    const double jac_tol = 1e-7;

    // Test rotation() / attitude() Jacobian
    Matrix Hrot_ana;
    kTestGal3.rotation(Hrot_ana);
    std::function<Rot3(const Gal3&)> rot_func =
        [](const Gal3& g) { return g.rotation(); };
    Matrix Hrot_num = numericalDerivative11(rot_func, kTestGal3, jac_tol);
    EXPECT(assert_equal(Hrot_num, Hrot_ana, jac_tol));

    // Test translation() / position() Jacobian
    Matrix Hpos_ana;
    kTestGal3.translation(Hpos_ana);
    std::function<Point3(const Gal3&)> pos_func =
        [](const Gal3& g) { return g.translation(); };
    Matrix Hpos_num = numericalDerivative11(pos_func, kTestGal3, jac_tol);
    EXPECT(assert_equal(Hpos_num, Hpos_ana, jac_tol));

    // Test velocity() Jacobian
    Matrix Hvel_ana;
    kTestGal3.velocity(Hvel_ana);
    std::function<Velocity3(const Gal3&)> vel_func =
        [](const Gal3& g) { return g.velocity(); };
    Matrix Hvel_num = numericalDerivative11(vel_func, kTestGal3, jac_tol);
    EXPECT(assert_equal(Hvel_num, Hvel_ana, jac_tol));

    // Test time() Jacobian
    Matrix Htime_ana;
    kTestGal3.time(Htime_ana);
    std::function<double(const Gal3&)> time_func =
        [](const Gal3& g) { return g.time(); };
    Matrix Htime_num = numericalDerivative11(time_func, kTestGal3, jac_tol);
    EXPECT(assert_equal(Htime_num, Htime_ana, jac_tol));
}

/* ************************************************************************* */
TEST(Gal3, Jacobian_Interpolate) {
    // *** CIRCULARITY WARNING ***
    // Gal3::interpolate computes its Jacobians using a chain rule involving
    // Logmap/Expmap Jacobians. Those are currently numerical, so this test verifies
    // the chain rule implementation assuming the underlying derivatives are correct.
    const double jac_tol = 1e-7;
    const Gal3 g1 = kTestGal3_Lie1;
    const Gal3 g2 = kTestGal3_Lie2;
    const double alpha = 0.3;

    Matrix H1_ana, H2_ana, Ha_ana;
    interpolate(g1, g2, alpha, H1_ana, H2_ana, Ha_ana);

    std::function<Gal3(const Gal3&, const Gal3&, const double&)> interp_func =
        [](const Gal3& g1_in, const Gal3& g2_in, const double& a){
            return interpolate(g1_in, g2_in, a); // Call without Jacobians
        };

    const double& alpha_ref = alpha; // Needed for lambda capture if using C++11/14
    Matrix H1_num = numericalDerivative31(interp_func, g1, g2, alpha_ref, jac_tol);
    Matrix H2_num = numericalDerivative32(interp_func, g1, g2, alpha_ref, jac_tol);
    Matrix Ha_num = numericalDerivative33(interp_func, g1, g2, alpha_ref, jac_tol);

    EXPECT(assert_equal(H1_num, H1_ana, jac_tol));
    EXPECT(assert_equal(H2_num, H2_ana, jac_tol));
    EXPECT(assert_equal(Ha_num, Ha_ana, jac_tol));
}

/* ************************************************************************* */
TEST(Gal3, StaticAdjoint) {
    // Verify static adjointMap and adjoint Jacobians against numerical derivatives
    // and analytical properties. Assumes static adjoint provides analytical Jacobians.
    const double jac_tol = 1e-7;
    Vector10 xi = (Vector10() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.1, 0.15, -0.25, 0.9).finished();
    Vector10 y = (Vector10() << -0.5, 0.4, -0.3, 0.2, -0.1, 0.0, 0.3, -0.35, 0.45, -0.1).finished();

    // Test static adjointMap
    Gal3::Jacobian ad_xi_map = Gal3::adjointMap(xi);
    Vector10 ad_xi_map_times_y = ad_xi_map * y;
    Vector10 ad_xi_y_direct = Gal3::adjoint(xi, y);
    EXPECT(assert_equal(ad_xi_map_times_y, ad_xi_y_direct, kTol)); // Check ad(xi)*y == [xi,y]

    // Verify d(adjoint(xi,y))/dy == adjointMap(xi) numerically
    std::function<Vector10(const Vector10&, const Vector10&)> static_adjoint_func =
        [](const Vector10& xi_in, const Vector10& y_in){
            return Gal3::adjoint(xi_in, y_in); // Call without Jacobians
        };
    Matrix Hy_num = numericalDerivative22(static_adjoint_func, xi, y, jac_tol);
    EXPECT(assert_equal(ad_xi_map, Hy_num, jac_tol));

    // Test static adjoint Jacobians
    Matrix Hxi_ana, Hy_ana;
    Gal3::adjoint(xi, y, Hxi_ana, Hy_ana); // Get analytical Jacobians

    EXPECT(assert_equal(ad_xi_map, Hy_ana, kTol)); // Check analytical Hy vs ad(xi)
    EXPECT(assert_equal(-Gal3::adjointMap(y), Hxi_ana, kTol)); // Check analytical Hxi vs -ad(y)

    // Verify analytical Hxi against numerical derivative
    Matrix Hxi_num = numericalDerivative21(static_adjoint_func, xi, y, jac_tol);
    EXPECT(assert_equal(Hxi_num, Hxi_ana, jac_tol));

    // Verify Jacobi Identity: [x, [y, z]] + [y, [z, x]] + [z, [x, y]] = 0
    Vector10 z = (Vector10() << 0.7, 0.8, 0.9, -0.1, -0.2, -0.3, 0.5, 0.6, 0.7, 0.1).finished();
    Vector10 term1 = Gal3::adjoint(xi, Gal3::adjoint(y, z));
    Vector10 term2 = Gal3::adjoint(y, Gal3::adjoint(z, xi));
    Vector10 term3 = Gal3::adjoint(z, Gal3::adjoint(xi, y));
    Vector10 sum_terms = term1 + term2 + term3;
    EXPECT(assert_equal(Vector10(Vector10::Zero()), sum_terms, jac_tol));
}

/* ************************************************************************* */
TEST(Gal3, ExpLog_NearZero) {
    const double kTolRoundtrip = kTol * 100; // Adjusted tolerance for round-trip

    // Small non-zero tangent vector 1
    Vector10 xi_small1;
    xi_small1 << 1e-5, -2e-5, 1.5e-5, -3e-5, 4e-5, -2.5e-5, 1e-7, -2e-7, 1.5e-7, -5e-5;
    Gal3 g_small1 = Gal3::Expmap(xi_small1);
    Vector10 xi_recovered1 = Gal3::Logmap(g_small1);
    EXPECT(assert_equal(xi_small1, xi_recovered1, kTolRoundtrip));

    // Small non-zero tangent vector 2
    Vector10 xi_small2;
    xi_small2 << -5e-6, 1e-6, -4e-6, 2e-6, -6e-6, 1e-6, -5e-8, 8e-8, -2e-8, 9e-6;
    Gal3 g_small2 = Gal3::Expmap(xi_small2);
    Vector10 xi_recovered2 = Gal3::Logmap(g_small2);
    EXPECT(assert_equal(xi_small2, xi_recovered2, kTolRoundtrip));

    // Even smaller theta magnitude
    Vector10 xi_small3;
    xi_small3 << 1e-9, 2e-9, 3e-9, -4e-9,-5e-9,-6e-9, 1e-10, 2e-10, 3e-10, 7e-9;
    Gal3 g_small3 = Gal3::Expmap(xi_small3);
    Vector10 xi_recovered3 = Gal3::Logmap(g_small3);
    EXPECT(assert_equal(xi_small3, xi_recovered3, kTol)); // Tighter tolerance near zero

    // Zero tangent vector (Strict Identity case)
    Vector10 xi_zero = Vector10::Zero();
    Gal3 g_identity = Gal3::Expmap(xi_zero);
    Vector10 xi_recovered_zero = Gal3::Logmap(g_identity);
    EXPECT(assert_equal(Gal3::Identity(), g_identity, kTol));
    EXPECT(assert_equal(xi_zero, xi_recovered_zero, kTol));
    EXPECT(assert_equal(xi_zero, Gal3::Logmap(Gal3::Expmap(xi_zero)), kTol));
}

//******************************************************************************
TEST(Gal3, Vec) {
    // Create a non-trivial Gal3 object
    const Rot3 R_test = Rot3::Rodrigues(0.1, 0.2, 0.3);
    const Point3 r_test(1.0, 2.0, 3.0);
    const Velocity3 v_test(0.4, 0.5, 0.6);
    const double t_test = 0.7;
    const Gal3 gal3(R_test, r_test, v_test, t_test);

    // 1. Test the Value
    const Matrix5 T = gal3.matrix();

    using Vector25 = Eigen::Matrix<double, 25, 1>;
    const Vector25 expected_vec = Eigen::Map<const Vector25>(T.data());
    Vector25 actual_vec = gal3.vec();
    EXPECT(assert_equal(expected_vec, actual_vec, 1e-9));

    // 2. Test the Jacobian
    Eigen::Matrix<double, 25, 10> H_actual;
    gal3.vec(H_actual);
    auto vec_fun = [](const Gal3& g) -> Vector25 {
        return g.vec();
        };
    Matrix H_numerical = numericalDerivative11<Vector25, Gal3, 10>(vec_fun, gal3);
    EXPECT(assert_equal(H_numerical, H_actual, 1e-7));
}

//******************************************************************************
TEST(Gal3, AdjointMap) {
  // Create a non-trivial Gal3 object
  const Rot3 R = Rot3::Rodrigues(0.1, 0.2, 0.3);
  const Point3 r(1.0, 2.0, 3.0);
  const Velocity3 v(0.4, 0.5, 0.6);
  const double t = 0.7;
  const Gal3 gal3(R, r, v, t);

  // Call the specialized AdjointMap
  Gal3::Jacobian specialized_Adj = gal3.AdjointMap();

  // Call the generic AdjointMap from the base class
  Gal3::Jacobian generic_Adj = static_cast<const MatrixLieGroup<Gal3, 10, 5>*>(&gal3)->AdjointMap();

  // Assert that they are equal
  EXPECT(assert_equal(specialized_Adj, generic_Adj, kTol));
}

/* ************************************************************************* */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
