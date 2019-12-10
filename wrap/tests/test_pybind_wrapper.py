"""
Unit test for Pybind wrap program
Author: Matthew Sklar
Date: February 2019
"""

import os
import sys
import unittest
import filecmp

import os.path as path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.normpath(os.path.abspath(os.path.join(__file__, '../../../build/wrap'))))

from pybind_wrapper import PybindWrapper
import gtsam_py.gtsam as gtsam
import interface_parser as parser
import template_instantiator as instantiator

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestWrap(unittest.TestCase):
    TEST_DIR = path.dirname(__file__)

    def test_geometry_python(self):
        """
        Check generation of python geometry wrapper.
        python3 ../pybind_wrapper.py --src geometry.h --module_name
            geometry_py --out output/geometry_py.cc"
        """
        with open(os.path.join(self.TEST_DIR, 'geometry.h'), 'r') as f:
            content = f.read()

        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        # Create Pybind wrapper instance
        wrapper = PybindWrapper(
            module=module,
            module_name='geometry_py',
            use_boost=False,
            top_module_namespaces=[''],
            ignore_classes=[''],
        )

        cc_content = wrapper.wrap()

        output = path.join(self.TEST_DIR, 'actual-python/geometry_py.cpp')

        if not path.exists(path.join(self.TEST_DIR, 'actual-python')):
            os.mkdir(path.join(self.TEST_DIR, 'actual-python'))

        with open(output, 'w') as f:
            f.write(cc_content)

        self.assertTrue(filecmp.cmp(
            output,
            path.join(self.TEST_DIR, 'expected-python/geometry_pybind.cpp'))
        )

    def test_gtsam(self):
        """Test insertion into and serialization/deserialization for gtsam
        KeySet object. This is primarily to test serialization, not KeySet.
        """
        x = gtsam.KeySet()

        self.assertTrue(x.size() == 0)

        x.insert(1)
        x.insert(1)
        x.insert(2)

        self.assertTrue(x.size() == 2)

        y = gtsam.KeySet()

        y = y.deserialize(x.serialize())

        self.assertTrue(x.equals(y))

if __name__ == '__main__':
    unittest.main()
