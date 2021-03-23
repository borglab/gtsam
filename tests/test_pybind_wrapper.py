"""
Unit test for Pybind wrap program
Author: Matthew Sklar, Varun Agrawal
Date: February 2019
"""

# pylint: disable=import-error, wrong-import-position, too-many-branches

import filecmp
import os
import os.path as path
import sys
import unittest

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.normpath(os.path.abspath(os.path.join(__file__, '../../../build/wrap'))))

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator
from gtwrap.pybind_wrapper import PybindWrapper

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestWrap(unittest.TestCase):
    """Tests for Python wrapper based on Pybind11."""
    TEST_DIR = os.path.dirname(os.path.realpath(__file__)) + "/"

    def wrap_content(self, content, module_name, output_dir):
        """
        Common function to wrap content.
        """
        module = parser.Module.parseString(content)

        instantiator.instantiate_namespace_inplace(module)

        with open(self.TEST_DIR + "pybind_wrapper.tpl") as template_file:
            module_template = template_file.read()

        # Create Pybind wrapper instance
        wrapper = PybindWrapper(
            module=module,
            module_name=module_name,
            use_boost=False,
            top_module_namespaces=[''],
            ignore_classes=[''],
            module_template=module_template
        )

        cc_content = wrapper.wrap()

        output = path.join(self.TEST_DIR, output_dir, module_name + ".cpp")

        if not path.exists(path.join(self.TEST_DIR, output_dir)):
            os.mkdir(path.join(self.TEST_DIR, output_dir))

        with open(output, 'w') as f:
            f.write(cc_content)

        return output

    def test_geometry_python(self):
        """
        Check generation of python geometry wrapper.
        python3 ../pybind_wrapper.py --src geometry.h --module_name
            geometry_py --out output/geometry_py.cc
        """
        with open(os.path.join(self.TEST_DIR, 'geometry.h'), 'r') as f:
            content = f.read()

        output = self.wrap_content(content, 'geometry_py', 'actual-python')

        expected = path.join(self.TEST_DIR, 'expected-python/geometry_pybind.cpp')
        success = filecmp.cmp(output, expected)

        if not success:
            os.system("diff {} {}".format(output, expected))
        self.assertTrue(success)

    def test_namespaces(self):
        """
        Check generation of python geometry wrapper.
        python3 ../pybind_wrapper.py --src testNamespaces.h --module_name
            testNamespaces_py --out output/testNamespaces_py.cc
        """
        with open(os.path.join(self.TEST_DIR, 'testNamespaces.h'), 'r') as f:
            content = f.read()

        output = self.wrap_content(content, 'testNamespaces_py', 'actual-python')

        expected = path.join(self.TEST_DIR, 'expected-python/testNamespaces_py.cpp')
        success = filecmp.cmp(output, expected)

        if not success:
            os.system("diff {} {}".format(output, expected))
        self.assertTrue(success)

if __name__ == '__main__':
    unittest.main()
