"""
Unit test for Matlab wrap program
Author: Matthew Sklar
Date: March 2019
"""
import os
import sys
import unittest
import filecmp

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import template_instantiator as instantiator
import interface_parser as parser
from matlab_wrapper import MatlabWrapper


class TestWrap(unittest.TestCase):
    TEST_DIR = "wrap/tests/"
    MATLAB_TEST_DIR = TEST_DIR + "expected-matlab/"
    MATLAB_ACTUAL_DIR = TEST_DIR + "actual-matlab/"

    def _generate_content(self, cc_content, path=''):
        """Generate files and folders from matlab wrapper content

        Keyword arguments:
        cc_content -- the content to generate formatted as
            (file_name, file_content) or
            (folder_name, [(file_name, file_content)])
        path -- the path to the files parent folder within the main folder
        """
        for c in cc_content:
            if type(c) == list:
                path_to_folder = self.MATLAB_ACTUAL_DIR if path == '' else path
                path_to_folder += c[0][0]

                if not os.path.isdir(path_to_file):
                    try:
                        os.mkdir(path_to_folder)
                    except OSError:
                        print("Failed to create directory {path}".format(
                            path=path_to_file))

                for sub_content in c:
                    self._generate_content(sub_content[1], path_to_folder)
            else:
                path_to_file = self.MATLAB_ACTUAL_DIR + c[0] if path == '' \
                    else path + '/' + c[0]

                with open(path_to_file, 'w') as f:
                    f.write(c[1])

    def test_geometry_matlab(self):
        """ Check generation of matlab geometry wrapper.
        python3 wrap/matlab_wrapper.py --src wrap/tests/geometry.h
            --module_name geometry --out wrap/tests/actual-matlab
        """
        with open(self.TEST_DIR + 'geometry.h', 'r') as f:
            content = f.read()

        if not os.path.exists(self.MATLAB_ACTUAL_DIR):
            os.mkdir(self.MATLAB_ACTUAL_DIR)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        # Create MATLAB wrapper instance
        wrapper = MatlabWrapper(
            module=module,
            module_name='geometry',
            top_module_namespace=[''],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self._generate_content(cc_content)

        self.assertTrue(os.path.isdir(self.MATLAB_ACTUAL_DIR + '+gtsam'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + '+gtsam/Point2.m',
            self.MATLAB_TEST_DIR + '+gtsam/Point2.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + '+gtsam/Point3.m',
            self.MATLAB_TEST_DIR + '+gtsam/Point3.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'Test.m',
            self.MATLAB_TEST_DIR + 'Test.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'MyBase.m',
            self.MATLAB_TEST_DIR + 'MyBase.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'MyTemplatePoint2.m',
            self.MATLAB_TEST_DIR + 'MyTemplatePoint2.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'MyTemplateMatrix.m',
            self.MATLAB_TEST_DIR + 'MyTemplateMatrix.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'MyVector3.m',
            self.MATLAB_TEST_DIR + 'MyVector3.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'MyVector12.m',
            self.MATLAB_TEST_DIR + 'MyVector12.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'MyFactorPosePoint2.m',
            self.MATLAB_TEST_DIR + 'MyFactorPosePoint2.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'aGlobalFunction.m',
            self.MATLAB_TEST_DIR + 'aGlobalFunction.m'))
        self.assertTrue(filecmp.cmp(
            self.MATLAB_ACTUAL_DIR + 'overloadedGlobalFunction.m',
            self.MATLAB_TEST_DIR + 'overloadedGlobalFunction.m'))

if __name__ == '__main__':
    unittest.main()
