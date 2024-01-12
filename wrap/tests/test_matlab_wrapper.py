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
            'FunDouble.m',
            'FunRange.m',
            'MultipleTemplatesIntDouble.m',
            'MultipleTemplatesIntFloat.m',
            'MyFactorPosePoint2.m',
            'MyVector3.m',
            'MyVector12.m',
            'PrimitiveRefDouble.m',
            'Test.m',
            'ForwardKinematics.m',
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
            'MyTemplateMatrix.m',
            'MyTemplatePoint2.m',
            'ForwardKinematicsFactor.m',
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
