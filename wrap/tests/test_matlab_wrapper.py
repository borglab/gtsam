"""
Unit tests for Matlab wrap program
Author: Matthew Sklar, Varun Agrawal
Date: March 2019
"""
# pylint: disable=import-error, wrong-import-position, too-many-branches

import filecmp
import os
import sys
import unittest

from loguru import logger

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator
from gtwrap.matlab_wrapper import MatlabWrapper


class TestWrap(unittest.TestCase):
    """
    Test the Matlab wrapper
    """
    TEST_DIR = os.path.dirname(os.path.realpath(__file__)) + "/"
    MATLAB_TEST_DIR = TEST_DIR + "expected-matlab/"
    MATLAB_ACTUAL_DIR = TEST_DIR + "actual-matlab/"

    # set the log level to INFO by default
    logger.remove()  # remove the default sink
    logger.add(sys.stderr, format="{time} {level} {message}", level="INFO")

    def generate_content(self, cc_content, path=''):
        """Generate files and folders from matlab wrapper content.

        Keyword arguments:
        cc_content -- the content to generate formatted as
            (file_name, file_content) or
            (folder_name, [(file_name, file_content)])
        path -- the path to the files parent folder within the main folder
        """
        if path == '':
            path = self.MATLAB_ACTUAL_DIR

        for c in cc_content:
            if isinstance(c, list):
                if len(c) == 0:
                    continue
                logger.debug("c object: {}".format(c[0][0]))
                path_to_folder = path + '/' + c[0][0]

                if not os.path.isdir(path_to_folder):
                    try:
                        os.makedirs(path_to_folder, exist_ok=True)
                    except OSError:
                        pass

                for sub_content in c:
                    logger.debug("sub object: {}".format(sub_content[1][0][0]))
                    self.generate_content(sub_content[1], path_to_folder)

            elif isinstance(c[1], list):
                path_to_folder = path + '/' + c[0]

                logger.debug("[generate_content_global]: {}".format(path_to_folder))
                if not os.path.isdir(path_to_folder):
                    try:
                        os.makedirs(path_to_folder, exist_ok=True)
                    except OSError:
                        pass
                for sub_content in c[1]:
                    path_to_file = path_to_folder + '/' + sub_content[0]
                    logger.debug(
                        "[generate_global_method]: {}".format(path_to_file))
                    with open(path_to_file, 'w') as f:
                        f.write(sub_content[1])

            else:
                path_to_file = path + '/' + c[0]

                logger.debug("[generate_content]: {}".format(path_to_file))
                if not os.path.isdir(path_to_file):
                    try:
                        os.mkdir(path)
                    except OSError:
                        pass

                with open(path_to_file, 'w') as f:
                    f.write(c[1])

    def test_geometry_matlab(self):
        """
        Check generation of matlab geometry wrapper.
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
            top_module_namespace=['gtsam'],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        self.generate_content(cc_content)

        def compare_and_diff(file):
            output = self.MATLAB_ACTUAL_DIR + file
            expected = self.MATLAB_TEST_DIR + file
            success = filecmp.cmp(output, expected)
            if not success:
                print("Differ in file: {}".format(file))
                os.system("diff {} {}".format(output, expected))
            self.assertTrue(success)

        self.assertTrue(os.path.isdir(self.MATLAB_ACTUAL_DIR + '+gtsam'))

        files = [
            '+gtsam/Point2.m', '+gtsam/Point3.m', 'Test.m', 'MyBase.m',
            'load2D.m', 'MyTemplatePoint2.m', 'MyTemplateMatrix.m',
            'MyVector3.m', 'MyVector12.m', 'MyFactorPosePoint2.m',
            'aGlobalFunction.m', 'overloadedGlobalFunction.m',
            'geometry_wrapper.cpp'
        ]

        for file in files:
            compare_and_diff(file)


if __name__ == '__main__':
    unittest.main()
