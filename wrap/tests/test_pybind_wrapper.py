"""
Unit test for Pybind wrap program
Author: Matthew Sklar
Date: February 2019
"""

import os
import sys
import unittest
import filecmp

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pybind_wrapper import PybindWrapper
import interface_parser as parser
import template_instantiator as instantiator


class TestWrap(unittest.TestCase):
    TEST_DIR = "wrap/tests/"

    def test_geometry_python(self):
        """
        Check generation of python geometry wrapper.
        python3 ../pybind_wrapper.py --src geometry.h --module_name
            geometry_py --out output/geometry_py.cc"
        """
        with open(self.TEST_DIR + 'geometry.h', 'r') as f:
            content = f.read()

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        wrapper = PybindWrapper(
            module=module,
            module_name='geometry_py',
            use_boost=False,
            top_module_namespaces=[''],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        output = self.TEST_DIR + 'actual-python/geometry_py.cpp'

        if not os.path.exists(self.TEST_DIR + 'actual-python'):
            os.mkdir(self.TEST_DIR + 'actual-python')

        with open(self.TEST_DIR + 'actual-python/geometry_py.cpp', 'w') as f:
            f.write(cc_content)

        self.assertTrue(filecmp.cmp(
            output, self.TEST_DIR + 'expected-python/geometry_pybind.cpp'))


if __name__ == '__main__':
    unittest.main()
