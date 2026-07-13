"""
Unit test for Pybind wrap program
Author: Matthew Sklar, Varun Agrawal
Date: February 2019
"""

# pylint: disable=import-error, wrong-import-position, too-many-branches

import filecmp
import os
import os.path as osp
import sys
import unittest

sys.path.append(osp.dirname(osp.dirname(osp.abspath(__file__))))
sys.path.append(
    osp.normpath(osp.abspath(osp.join(__file__, '../../../build/wrap'))))

from gtwrap.pybind_wrapper import PybindWrapper

sys.path.append(osp.dirname(osp.dirname(osp.abspath(__file__))))


class TestWrap(unittest.TestCase):
    """Tests for Python wrapper based on Pybind11."""
    TEST_DIR = osp.dirname(osp.realpath(__file__))
    INTERFACE_DIR = osp.join(TEST_DIR, 'fixtures')
    PYTHON_TEST_DIR = osp.join(TEST_DIR, 'expected', 'python')
    PYTHON_ACTUAL_DIR = osp.join(TEST_DIR, "actual", "python")

    # Create the `actual/python` directory
    os.makedirs(PYTHON_ACTUAL_DIR, exist_ok=True)

    def wrap_content(self,
                     sources,
                     module_name,
                     output_dir,
                     use_boost_serialization=False,
                     module_template=None):
        """
        Common function to wrap content in `sources`.
        """
        if module_template is None:
            with open(osp.join(self.TEST_DIR, "pybind_wrapper.tpl"),
                      encoding="UTF-8") as template_file:
                module_template = template_file.read()

        # Create Pybind wrapper instance
        wrapper = PybindWrapper(
            module_name=module_name,
            top_module_namespaces=[''],
            ignore_classes=[''],
            module_template=module_template,
            use_boost_serialization=use_boost_serialization)

        output = osp.join(self.TEST_DIR, output_dir, module_name + ".cpp")

        if not osp.exists(osp.join(self.TEST_DIR, output_dir)):
            os.mkdir(osp.join(self.TEST_DIR, output_dir))

        wrapper.wrap(sources, output)

        return output

    def compare_and_diff(self, file, actual):
        """
        Compute the comparison between the expected and actual file,
        and assert if diff is zero.
        """
        expected = osp.join(self.PYTHON_TEST_DIR, file)
        success = filecmp.cmp(actual, expected)

        if not success:
            os.system(f"diff {actual} {expected}")
        self.assertTrue(success, f"Mismatch for file {file}")

    def test_geometry(self):
        """
        Check generation of python geometry wrapper.
        python3 ../pybind_wrapper.py --src geometry.h --module_name
            geometry_py --out output/geometry_py.cc
        """
        source = osp.join(self.INTERFACE_DIR, 'geometry.i')
        output = self.wrap_content([source],
                                   'geometry_py',
                                   self.PYTHON_ACTUAL_DIR,
                                   use_boost_serialization=True)

        self.compare_and_diff('geometry_pybind.cpp', output)

    def test_functions(self):
        """Test interface file with function info."""
        source = osp.join(self.INTERFACE_DIR, 'functions.i')
        output = self.wrap_content([source], 'functions_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('functions_pybind.cpp', output)

    def test_class(self):
        """Test interface file with only class info."""
        source = osp.join(self.INTERFACE_DIR, 'class.i')
        output = self.wrap_content([source], 'class_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('class_pybind.cpp', output)

    def test_templates(self):
        """Test interface file with templated class."""
        source = osp.join(self.INTERFACE_DIR, 'templates.i')
        output = self.wrap_content([source], 'templates_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('templates_pybind.cpp', output)

    def test_inheritance(self):
        """Test interface file with class inheritance definitions."""
        source = osp.join(self.INTERFACE_DIR, 'inheritance.i')
        output = self.wrap_content([source], 'inheritance_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('inheritance_pybind.cpp', output)

    def test_namespaces(self):
        """
        Check generation of python wrapper for full namespace definition.
        python3 ../pybind_wrapper.py --src namespaces.i --module_name
            namespaces_py --out output/namespaces_py.cpp
        """
        source = osp.join(self.INTERFACE_DIR, 'namespaces.i')
        output = self.wrap_content([source], 'namespaces_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('namespaces_pybind.cpp', output)

    def test_operator_overload(self):
        """
        Tests for operator overloading.
        """
        source = osp.join(self.INTERFACE_DIR, 'operator.i')
        output = self.wrap_content([source], 'operator_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('operator_pybind.cpp', output)

    def test_special_cases(self):
        """
        Tests for some unique, non-trivial features.
        """
        source = osp.join(self.INTERFACE_DIR, 'special_cases.i')
        output = self.wrap_content([source], 'special_cases_py',
                                   self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('special_cases_pybind.cpp', output)

    def test_enum(self):
        """
        Test if enum generation is correct.
        """
        source = osp.join(self.INTERFACE_DIR, 'enum.i')
        output = self.wrap_content([source], 'enum_py', self.PYTHON_ACTUAL_DIR)

        self.compare_and_diff('enum_pybind.cpp', output)

    def test_argument_policy_hook(self):
        """Test that typed args are emitted through the overridable policy hook."""
        source = osp.join(self.INTERFACE_DIR, 'arg_policies.i')
        module_template = """#include <pybind11/pybind11.h>

{includes}

#include "python/arg_policy_preamble.h"

namespace py = pybind11;

PYBIND11_MODULE({module_name}, m_) {{
    m_.doc() = "pybind11 wrapper of {module_name}";

{wrapped_namespace}

}}
"""
        output = self.wrap_content([source],
                                   'arg_policies_py',
                                   self.PYTHON_ACTUAL_DIR,
                                   module_template=module_template)

        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()

        self.assertLess(content.index('template <typename T>\nstruct PyArgPolicy'),
                        content.index('#include "python/arg_policy_preamble.h"'))
        self.assertIn(
            'gtwrap::internal::py_arg<const testing::SpecialView&>("values")',
            content)
        self.assertIn('gtwrap::internal::py_arg<int>("count") = 1', content)

    def test_const_ref_return_policy(self):
        """Test that methods returning const T& emit reference_internal policy.

        Without this policy, pybind11 defaults to copying the returned reference.
        With the policy, the binding keeps the reference alive via the parent object.

        Expected emitted code difference:
          Before: [](Cls* self, ...){return self->method(...);}, py::arg(...))
          After:  [](Cls* self, ...) -> const auto&{return self->method(...);},
                  py::return_value_policy::reference_internal, py::arg(...))
        """
        source = osp.join(self.INTERFACE_DIR, 'class.i')
        output = self.wrap_content([source], 'class_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r') as f:
            content = f.read()

        # const Vector& return_vector2 should have reference_internal
        self.assertIn('-> const auto&{return self->return_vector2', content)
        self.assertIn('py::return_value_policy::reference_internal', content)

        # const Matrix& return_matrix2 should also have reference_internal
        self.assertIn('-> const auto&{return self->return_matrix2', content)

        # Non-ref returns (e.g. return_vector1 which returns by value) should NOT
        lines = content.split('\n')
        for line in lines:
            if 'return_vector1' in line:
                self.assertNotIn('reference_internal', line)
                self.assertNotIn('-> const auto&', line)
            if 'return_matrix1' in line:
                self.assertNotIn('reference_internal', line)
                self.assertNotIn('-> const auto&', line)

        source = osp.join(self.INTERFACE_DIR, 'return_policies.i')
        output = self.wrap_content([source], 'return_policies_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()

        for line in content.split('\n'):
            if 'return_const_ref' in line:
                self.assertIn('reference_internal', line)
                self.assertIn('-> const auto&', line)
            if '.def("return_mutable_ref"' in line or '.def("return_value"' in line:
                self.assertNotIn('reference_internal', line)
                self.assertNotIn('-> const auto&', line)


if __name__ == '__main__':
    unittest.main()
