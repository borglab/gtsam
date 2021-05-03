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

from loguru import logger

sys.path.append(osp.dirname(osp.dirname(osp.abspath(__file__))))

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator
from gtwrap.matlab_wrapper import MatlabWrapper


class TestWrap(unittest.TestCase):
    """
    Test the Matlab wrapper
    """
    TEST_DIR = osp.dirname(osp.realpath(__file__))
    INTERFACE_DIR = osp.join(TEST_DIR, "fixtures")
    MATLAB_TEST_DIR = osp.join(TEST_DIR, "expected", "matlab")
    MATLAB_ACTUAL_DIR = osp.join(TEST_DIR, "actual", "matlab")

    # Create the `actual/matlab` directory
    os.makedirs(MATLAB_ACTUAL_DIR, exist_ok=True)

    # set the log level to INFO by default
    logger.remove()  # remove the default sink
    logger.add(sys.stderr, format="{time} {level} {message}", level="INFO")

    def generate_content(self, cc_content, path=MATLAB_ACTUAL_DIR):
        """Generate files and folders from matlab wrapper content.

        Keyword arguments:
        cc_content -- the content to generate formatted as
            (file_name, file_content) or
            (folder_name, [(file_name, file_content)])
        path -- the path to the files parent folder within the main folder
        """
        for c in cc_content:
            if isinstance(c, list):
                if len(c) == 0:
                    continue
                logger.debug("c object: {}".format(c[0][0]))
                path_to_folder = osp.join(path, c[0][0])

                if not osp.isdir(path_to_folder):
                    try:
                        os.makedirs(path_to_folder, exist_ok=True)
                    except OSError:
                        pass

                for sub_content in c:
                    logger.debug("sub object: {}".format(sub_content[1][0][0]))
                    self.generate_content(sub_content[1], path_to_folder)

            elif isinstance(c[1], list):
                path_to_folder = osp.join(path, c[0])

                logger.debug(
                    "[generate_content_global]: {}".format(path_to_folder))
                if not osp.isdir(path_to_folder):
                    try:
                        os.makedirs(path_to_folder, exist_ok=True)
                    except OSError:
                        pass
                for sub_content in c[1]:
                    path_to_file = osp.join(path_to_folder, sub_content[0])
                    logger.debug(
                        "[generate_global_method]: {}".format(path_to_file))
                    with open(path_to_file, 'w') as f:
                        f.write(sub_content[1])

            else:
                path_to_file = osp.join(path, c[0])

                logger.debug("[generate_content]: {}".format(path_to_file))
                if not osp.isdir(path_to_file):
                    try:
                        os.mkdir(path)
                    except OSError:
                        pass

                with open(path_to_file, 'w') as f:
                    f.write(c[1])

    def compare_and_diff(self, file):
        """
        Compute the comparison between the expected and actual file,
        and assert if diff is zero.
        """
        output = osp.join(self.MATLAB_ACTUAL_DIR, file)
        expected = osp.join(self.MATLAB_TEST_DIR, file)
        success = filecmp.cmp(output, expected)
        if not success:
            print("Differ in file: {}".format(file))
            os.system("diff {} {}".format(output, expected))
        self.assertTrue(success, "Mismatch for file {0}".format(file))

    def test_geometry(self):
        """
        Check generation of matlab geometry wrapper.
        python3 wrap/matlab_wrapper.py --src wrap/tests/geometry.h
            --module_name geometry --out wrap/tests/actual-matlab
        """
        with open(osp.join(self.INTERFACE_DIR, 'geometry.i'), 'r') as f:
            content = f.read()

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        # Create MATLAB wrapper instance
        wrapper = MatlabWrapper(
            module=module,
            module_name='geometry',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        self.assertTrue(osp.isdir(osp.join(self.MATLAB_ACTUAL_DIR, '+gtsam')))

        files = ['+gtsam/Point2.m', '+gtsam/Point3.m', 'geometry_wrapper.cpp']

        for file in files:
            self.compare_and_diff(file)

    def test_functions(self):
        """Test interface file with function info."""
        with open(osp.join(self.INTERFACE_DIR, 'functions.i'), 'r') as f:
            content = f.read()

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = MatlabWrapper(
            module=module,
            module_name='functions',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        files = [
            'functions_wrapper.cpp', 'aGlobalFunction.m', 'load2D.m',
            'MultiTemplatedFunctionDoubleSize_tDouble.m',
            'MultiTemplatedFunctionStringSize_tDouble.m',
            'overloadedGlobalFunction.m', 'TemplatedFunctionRot3.m'
        ]

        for file in files:
            self.compare_and_diff(file)

    def test_class(self):
        """Test interface file with only class info."""
        with open(osp.join(self.INTERFACE_DIR, 'class.i'), 'r') as f:
            content = f.read()

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = MatlabWrapper(
            module=module,
            module_name='class',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        files = [
            'class_wrapper.cpp', 'FunDouble.m', 'FunRange.m',
            'MultipleTemplatesIntDouble.m', 'MultipleTemplatesIntFloat.m',
            'MyFactorPosePoint2.m', 'MyVector3.m', 'MyVector12.m',
            'PrimitiveRefDouble.m', 'Test.m'
        ]

        for file in files:
            self.compare_and_diff(file)

    def test_inheritance(self):
        """Test interface file with class inheritance definitions."""
        with open(osp.join(self.INTERFACE_DIR, 'inheritance.i'), 'r') as f:
            content = f.read()

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = MatlabWrapper(
            module=module,
            module_name='inheritance',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        files = [
            'inheritance_wrapper.cpp', 'MyBase.m', 'MyTemplateMatrix.m',
            'MyTemplatePoint2.m'
        ]

        for file in files:
            self.compare_and_diff(file)

    def test_namspaces(self):
        """
        Test interface file with full namespace definition.
        """
        with open(osp.join(self.INTERFACE_DIR, 'namespaces.i'), 'r') as f:
            content = f.read()

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = MatlabWrapper(
            module=module,
            module_name='namespaces',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        files = [
            'namespaces_wrapper.cpp', '+ns1/aGlobalFunction.m',
            '+ns1/ClassA.m', '+ns1/ClassB.m', '+ns2/+ns3/ClassB.m',
            '+ns2/aGlobalFunction.m', '+ns2/ClassA.m', '+ns2/ClassC.m',
            '+ns2/overloadedGlobalFunction.m', 'ClassD.m'
        ]

        for file in files:
            self.compare_and_diff(file)

    def test_special_cases(self):
        """
        Tests for some unique, non-trivial features.
        """
        with open(osp.join(self.INTERFACE_DIR, 'special_cases.i'), 'r') as f:
            content = f.read()

        if not osp.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = MatlabWrapper(
            module=module,
            module_name='special_cases',
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        files = [
            'special_cases_wrapper.cpp',
            '+gtsam/PinholeCameraCal3Bundler.m',
            '+gtsam/NonlinearFactorGraph.m', 
        ]

        for file in files:
            self.compare_and_diff(file)

if __name__ == '__main__':
    unittest.main()
