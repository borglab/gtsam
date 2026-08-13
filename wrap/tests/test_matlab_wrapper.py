"""
Unit tests for Matlab wrap program
Author: Matthew Sklar, Varun Agrawal
Date: March 2019
"""
# pylint: disable=import-error, wrong-import-position

import filecmp
import os
import os.path as osp
import sys
import unittest

sys.path.append(osp.dirname(osp.dirname(osp.abspath(__file__))))

from gtwrap.matlab_wrapper import MatlabWrapper


class TestWrap(unittest.TestCase):
    """
    Test the Matlab wrapper
    """

    def setUp(self) -> None:
        super().setUp()

        # Set up all the directories
        self.TEST_DIR = osp.dirname(osp.realpath(__file__))
        self.INTERFACE_DIR = osp.join(self.TEST_DIR, "fixtures")
        self.MATLAB_TEST_DIR = osp.join(self.TEST_DIR, "expected", "matlab")
        self.MATLAB_ACTUAL_DIR = osp.join(self.TEST_DIR, "actual", "matlab")

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        # Generate the matlab.h file if it does not exist
        template_file = osp.join(self.TEST_DIR, "..", "gtwrap",
                                 "matlab_wrapper", "matlab_wrapper.tpl")
        if not osp.exists(template_file):
            with open(template_file, 'w', encoding="UTF-8") as tpl:
                tpl.write("#include <gtwrap/matlab.h>\n#include <map>\n")

        # Create the `actual/matlab` directory
        os.makedirs(self.MATLAB_ACTUAL_DIR, exist_ok=True)

    def compare_and_diff(self, file, actual):
        """
        Compute the comparison between the expected and actual file,
        and assert if diff is zero.
        """
        expected = osp.join(self.MATLAB_TEST_DIR, file)
        success = filecmp.cmp(actual, expected)

        if not success:
            os.system(f"diff {actual} {expected}")
        self.assertTrue(success, f"Mismatch for file {file}")

    def test_geometry(self):
        """
        Check generation of matlab geometry wrapper.
        python3 wrap/matlab_wrapper.py --src wrap/tests/geometry.h
            --module_name geometry --out wrap/tests/actual-matlab
        """
        file = osp.join(self.INTERFACE_DIR, 'geometry.i')

        # Create MATLAB wrapper instance
        wrapper = MatlabWrapper(module_name='geometry',
                                top_module_namespace=['gtsam'],
                                ignore_classes=[''],
                                use_boost_serialization=True)

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = ['+gtsam/Point2.m', '+gtsam/Point3.m', 'geometry_wrapper.cpp']

        self.assertTrue(osp.isdir(osp.join(self.MATLAB_ACTUAL_DIR, '+gtsam')))

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_matrix_view_arguments(self):
        """Test that matrix view arguments use MATLAB double arrays directly."""
        file = osp.join(self.INTERFACE_DIR, 'matrix_views.i')

        wrapper = MatlabWrapper(module_name='matrix_views',
                                top_module_namespace=['gtsam'],
                                ignore_classes=[''])

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        cpp_file = osp.join(self.MATLAB_ACTUAL_DIR, 'matrix_views_wrapper.cpp')
        with open(cpp_file, 'r', encoding='UTF-8') as f:
            cpp_content = f.read()

        self.assertIn(
            'gtsam::ConstMatrixView points = unwrapMatrixView< gtsam::ConstMatrixView >(in[1]);',
            cpp_content)
        self.assertIn('obj->acceptView(points);', cpp_content)
        self.assertIn('obj->scaleView(points,scale)', cpp_content)
        self.assertNotIn('unwrap< gtsam::ConstMatrixView >', cpp_content)
        self.assertNotIn('*points', cpp_content)

        m_file = osp.join(self.MATLAB_ACTUAL_DIR, '+gtsam',
                          'MatrixViewFixture.m')
        with open(m_file, 'r', encoding='UTF-8') as f:
            matlab_content = f.read()

        self.assertIn("isa(varargin{1},'double')", matlab_content)

        matlab_header = osp.join(self.TEST_DIR, '..', 'matlab.h')
        with open(matlab_header, 'r', encoding='UTF-8') as f:
            header_content = f.read()

        self.assertIn('unwrapMatrixView', header_content)
        self.assertIn('mxIsSparse(array)', header_content)
        self.assertIn('mwSize rows', header_content)
        self.assertIn('static_cast<unsigned long long>(rows)', header_content)
        self.assertIn('Eigen::Index m', header_content)
        self.assertIn('Stride(m, 1)', header_content)

    def test_eigen_ref_jacobians(self):
        """Test that Eigen::Ref<MatrixXd> args are treated as Jacobian outputs.

        Ref<MatrixXd> arguments should not appear as inputs in the MATLAB
        dispatch check, and should be returned as extra output arguments
        alongside the primary return value in the generated C++ MEX code.
        Covers: primitive inputs, zero inputs, class-type inputs, static methods.
        See https://github.com/borglab/gtsam/issues/2492
        """
        file = osp.join(self.INTERFACE_DIR, 'eigen_ref.i')

        wrapper = MatlabWrapper(module_name='eigen_ref',
                                top_module_namespace=['gtsam'],
                                ignore_classes=[''])

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        cpp_file = osp.join(self.MATLAB_ACTUAL_DIR, 'eigen_ref_wrapper.cpp')
        with open(cpp_file, 'r', encoding='UTF-8') as f:
            cpp_content = f.read()

        m_file = osp.join(self.MATLAB_ACTUAL_DIR, '+gtsam', 'Pose3.m')
        with open(m_file, 'r', encoding='UTF-8') as f:
            matlab_content = f.read()

        # Must never emit the bogus Eigen.RefMatrixXd MATLAB class check.
        self.assertNotIn('Eigen.RefMatrixXd', matlab_content)
        # Must never try to unwrap Ref args from in[].
        self.assertNotIn('unwrap_shared_ptr< Eigen::MatrixXd >', cpp_content)
        self.assertNotIn('unwrap< Eigen::MatrixXd >', cpp_content)

        # Case 1: transformFrom — primitive input (Point3) + 2 Ref args.
        # MATLAB: only 1 varargin (point), nargout == 3.
        self.assertIn(
            "length(varargin) == 1 && isa(varargin{1},'double')"
            " && size(varargin{1},1)==3 && size(varargin{1},2)==1"
            " && nargout == 3",
            matlab_content)
        # C++: allocates both Ref args, passes without dereference, returns out[1]/out[2].
        self.assertIn('Eigen::MatrixXd Hself = Eigen::MatrixXd();', cpp_content)
        self.assertIn('Eigen::MatrixXd Hpoint = Eigen::MatrixXd();', cpp_content)
        self.assertIn('obj->transformFrom(point,Hself,Hpoint)', cpp_content)
        self.assertIn('out[1] = wrap< Eigen::MatrixXd >(Hself);', cpp_content)
        self.assertIn('out[2] = wrap< Eigen::MatrixXd >(Hpoint);', cpp_content)
        self.assertIn('checkArguments("transformFrom",nargout,nargin-1,1);', cpp_content)

        # Case 2: inverse — zero real inputs + 1 Ref arg.
        # MATLAB: length(varargin) == 0, nargout == 2.
        self.assertIn('length(varargin) == 0 && nargout == 2', matlab_content)
        # C++: single Ref arg allocated and returned via out[1].
        self.assertIn('Eigen::MatrixXd H = Eigen::MatrixXd();', cpp_content)
        self.assertIn('obj->inverse(H)', cpp_content)
        self.assertIn('out[1] = wrap< Eigen::MatrixXd >(H);', cpp_content)
        self.assertIn('checkArguments("inverse",nargout,nargin-1,0);', cpp_content)

        # Case 3: between — class-type input (Pose3) + 2 Ref args.
        # MATLAB: 1 varargin (pose object), nargout == 3.
        self.assertIn(
            "length(varargin) == 1 && isa(varargin{1},'gtsam.Pose3') && nargout == 3",
            matlab_content)
        # C++: pose unwrapped correctly, H1/H2 allocated and returned.
        self.assertIn('Eigen::MatrixXd H1 = Eigen::MatrixXd();', cpp_content)
        self.assertIn('Eigen::MatrixXd H2 = Eigen::MatrixXd();', cpp_content)
        self.assertIn('obj->between(pose,H1,H2)', cpp_content)
        self.assertIn('out[1] = wrap< Eigen::MatrixXd >(H1);', cpp_content)
        self.assertIn('out[2] = wrap< Eigen::MatrixXd >(H2);', cpp_content)

        # Case 4: Expmap — static method + 1 Ref arg.
        # MATLAB: 1 varargin (xi), nargout == 2.
        self.assertIn(
            "length(varargin) == 1 && isa(varargin{1},'double')"
            " && size(varargin{1},2)==1 && nargout == 2",
            matlab_content)
        # C++: Hxi allocated and returned via out[1], nargin not decremented (static).
        self.assertIn('Eigen::MatrixXd Hxi = Eigen::MatrixXd();', cpp_content)
        self.assertIn('gtsam::Pose3::Expmap(xi,Hxi)', cpp_content)
        self.assertIn('out[1] = wrap< Eigen::MatrixXd >(Hxi);', cpp_content)
        self.assertIn('checkArguments("gtsam::Pose3.Expmap",nargout,nargin,1);', cpp_content)
        
    def test_functions(self):
        """Test interface file with function info."""
        file = osp.join(self.INTERFACE_DIR, 'functions.i')

        wrapper = MatlabWrapper(
            module_name='functions',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'functions_wrapper.cpp',
            'aGlobalFunction.m',
            'load2D.m',
            'MultiTemplatedFunctionDoubleSize_tDouble.m',
            'MultiTemplatedFunctionStringSize_tDouble.m',
            'overloadedGlobalFunction.m',
            'TemplatedFunctionRot3.m',
            'DefaultFuncInt.m',
            'DefaultFuncObj.m',
            'DefaultFuncString.m',
            'DefaultFuncVector.m',
            'DefaultFuncZero.m',
            'setPose.m',
            'EliminateDiscrete.m',
            'triangulatePoint3Cal3_S2.m',
            'FindKarcherMeanPoint3.m',
            'FindKarcherMeanSO3.m',
            'FindKarcherMeanSO4.m',
            'FindKarcherMeanPose3.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_class(self):
        """Test interface file with only class info."""
        file = osp.join(self.INTERFACE_DIR, 'class.i')

        wrapper = MatlabWrapper(
            module_name='class',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'class_wrapper.cpp',
            'ForwardKinematics.m',
            'FunDouble.m',
            'FunRange.m',
            'HessianFactor.m',
            'MultipleTemplatesIntDouble.m',
            'MultipleTemplatesIntFloat.m',
            'MyFactorPosePoint2.m',
            'MyVector3.m',
            'MyVector12.m',
            'PrimitiveRefDouble.m',
            'SmartProjectionRigFactorPinholeCameraCal3_S2.m',
            'Test.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_enum(self):
        """Test interface file with only enum info."""
        file = osp.join(self.INTERFACE_DIR, 'enum.i')

        wrapper = MatlabWrapper(
            module_name='enum',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'enum_wrapper.cpp',
            'Color.m',
            'Pet.m',
            '+Pet/Kind.m',
            '+gtsam/VerbosityLM.m',
            '+gtsam/+MCU/Avengers.m',
            '+gtsam/+MCU/GotG.m',
            '+gtsam/+OptimizerGaussNewtonParams/Verbosity.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_templates(self):
        """Test interface file with template info."""
        file = osp.join(self.INTERFACE_DIR, 'templates.i')

        wrapper = MatlabWrapper(
            module_name='template',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'template_wrapper.cpp', 'ScopedTemplateResult.m',
            'TemplatedConstructor.m'
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_inheritance(self):
        """Test interface file with class inheritance definitions."""
        file = osp.join(self.INTERFACE_DIR, 'inheritance.i')

        wrapper = MatlabWrapper(
            module_name='inheritance',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )
        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'inheritance_wrapper.cpp',
            'MyBase.m',
            'MyTemplateA.m',
            'MyTemplateMatrix.m',
            'MyTemplatePoint2.m',
            'ForwardKinematicsFactor.m',
            'ParentHasTemplateDouble.m',
            'Base.m',
            'Derived.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_namespaces(self):
        """
        Test interface file with full namespace definition.
        """
        file = osp.join(self.INTERFACE_DIR, 'namespaces.i')

        wrapper = MatlabWrapper(
            module_name='namespaces',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'namespaces_wrapper.cpp',
            '+ns1/aGlobalFunction.m',
            '+ns1/ClassA.m',
            '+ns1/ClassB.m',
            '+ns2/+ns3/ClassB.m',
            '+ns2/aGlobalFunction.m',
            '+ns2/ClassA.m',
            '+ns2/ClassC.m',
            '+ns2/overloadedGlobalFunction.m',
            'ClassD.m',
            '+gtsam/Values.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_special_cases(self):
        """
        Tests for some unique, non-trivial features.
        """
        file = osp.join(self.INTERFACE_DIR, 'special_cases.i')

        wrapper = MatlabWrapper(
            module_name='special_cases',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )
        wrapper.wrap([file], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'special_cases_wrapper.cpp',
            '+gtsam/GeneralSFMFactorCal3Bundler.m',
            '+gtsam/NonlinearFactorGraph.m',
            '+gtsam/PinholeCameraCal3Bundler.m',
            '+gtsam/SfmTrack.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)

    def test_multiple_files(self):
        """
        Test for when multiple interface files are specified.
        """
        file1 = osp.join(self.INTERFACE_DIR, 'part1.i')
        file2 = osp.join(self.INTERFACE_DIR, 'part2.i')

        wrapper = MatlabWrapper(
            module_name='multiple_files',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        wrapper.wrap([file1, file2], path=self.MATLAB_ACTUAL_DIR)

        files = [
            'multiple_files_wrapper.cpp',
            '+gtsam/Class1.m',
            '+gtsam/Class2.m',
            '+gtsam/ClassA.m',
        ]

        for file in files:
            actual = osp.join(self.MATLAB_ACTUAL_DIR, file)
            self.compare_and_diff(file, actual)


if __name__ == '__main__':
    unittest.main()
