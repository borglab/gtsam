"""
Unit test for Python wrap program
Author: Matthew Sklar
Date: February 2019
"""

import os
import sys
import unittest
import filecmp

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import template_instantiator as instantiator
import interface_parser as parser
from pybind_wrapper import PybindWrapper
from matlab_wrapper import MatlabWrapper

test_dir = "wrap/tests/"

matlab_test_dir = test_dir + "expected-matlab/"
matlab_actual_dir = test_dir + "actual-matlab/"


class TestWrap(unittest.TestCase):
    def test_geometry_python(self):
        """ 
        Check generation of python geometry wrapper. 
        python3 ../pybind_wrapper.py --src geometry.h --module_name geometry_py --out output/geometry_py.cc"
        """
        with open(test_dir + 'geometry.h', 'r') as f:
            content = f.read()

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = PybindWrapper(
            module=module,
            module_name='geometry_py',
            use_boost=False,
            top_module_namespaces=[''],
            ignore_classes=['']
        )

        cc_content = wrapper.wrap()
        output = test_dir + 'actual-python/geometry_py.cpp'

        if not os.path.exists(test_dir + 'actual-python'):
            os.mkdir(test_dir + 'actual-python')

        with open(test_dir + 'actual-python/geometry_py.cpp', 'w') as f:
            f.write(cc_content)

        self.assertTrue(filecmp.cmp(output, test_dir +
                                    'expected-python/geometry_pybind.cpp'))

    def test_geometry_matlab(self):
        """ Check generation of matlab geometry wrapper """
        with open(test_dir + 'geometry.h', 'r') as f:
            content = f.read()

        if not os.path.exists(matlab_actual_dir):
            os.mkdir(matlab_actual_dir)

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = MatlabWrapper(
            module=module,
            module_name='geometry',
            top_module_namespace=[''],
            ignore_classes=['']
        )

        wrapper.wrap()

        self.assertTrue(filecmp.cmp(
            matlab_actual_dir + 'aGlobalFunction.m', matlab_test_dir + 'aGlobalFunction.m'))
        self.assertTrue(filecmp.cmp(matlab_actual_dir + 'overloadedGlobalFunction.m',
                                    matlab_test_dir + 'overloadedGlobalFunction.m'))
        self.assertTrue(filecmp.cmp(matlab_actual_dir +
                                    'MyBase.m', matlab_test_dir + 'MyBase.m'))
        self.assertTrue(filecmp.cmp(
            matlab_actual_dir + 'MyFactorPosePoint2.m', matlab_test_dir + 'MyFactorPose2.m'))
        self.assertTrue(filecmp.cmp(
            matlab_actual_dir + 'MyTemplateMatrix.m', matlab_test_dir + 'MyTemplateMatrix.m'))
        self.assertTrue(filecmp.cmp(
            matlab_actual_dir + 'MyTemplatePoint2.m', matlab_test_dir + 'MyTemplatePoint2.m'))
        self.assertTrue(filecmp.cmp(matlab_actual_dir +
                                    'Test.m', matlab_test_dir + 'Test.m'))


if __name__ == '__main__':
    unittest.main()
