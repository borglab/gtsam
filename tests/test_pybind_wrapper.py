"""
Unit test for Pybind wrap program
Author: Matthew Sklar, Varun Agrawal
Date: February 2019
"""

# pylint: disable=import-error, wrong-import-position, too-many-branches

import filecmp
import os
import os.path as osp
import shlex
import shutil
import subprocess
import sys
import sysconfig
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

    MINIMAL_MODULE_TEMPLATE = """#include <pybind11/pybind11.h>

{includes}

using namespace std;
namespace py = pybind11;

{submodules}

{declaration_module_def} {{
{wrapped_declarations}
}}

{binding_module_def} {{
{wrapped_bindings}
}}

{module_def} {{
{module_init}
}}
"""

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
        success = filecmp.cmp(actual, expected, shallow=False)

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

    def test_serializable_template_typedef(self):
        """Serialization metadata applies to one template typedef only."""
        source = osp.join(self.INTERFACE_DIR, 'serializable_typedef.i')
        output = self.wrap_content([source],
                                   'serializable_typedef_py',
                                   self.PYTHON_ACTUAL_DIR,
                                   use_boost_serialization=True)
        with open(output, 'r', encoding='UTF-8') as generated:
            content = generated.read()

        self.assertEqual(1, content.count('.def("serialize"'))
        self.assertEqual(1, content.count('.def("deserialize"'))
        self.assertEqual(1, content.count('.def(py::pickle('))
        self.assertIn(
            'BOOST_CLASS_EXPORT(gtsam::SerializableTypedefFixture<int>)',
            content)
        self.assertNotIn(
            'BOOST_CLASS_EXPORT(gtsam::SerializableTypedefFixture<double>)',
            content)

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

        with open(output, 'r', encoding='UTF-8') as output_file:
            content = output_file.read()

        # Typedef template instantiations are moved to the end by the
        # instantiator. Their declarations must still precede methods which
        # use them in generated docstring signatures.
        class_declaration = (
            'py::class_<gtsam::PinholeCamera<gtsam::Cal3Bundler>, '
            'std::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>'
            '(m_gtsam, "PinholeCameraCal3Bundler");')
        self.assertLess(content.index(class_declaration),
                        content.index('.def("addPriorPinholeCameraCal3Bundler"'))

    def test_module_wide_declare_then_bind_order(self):
        """Declare every interface file before binding any of them."""
        main_source = osp.join(self.INTERFACE_DIR, 'declare_bind_main.i')
        late_source = osp.join(self.INTERFACE_DIR, 'declare_bind_late.i')
        output = self.wrap_content([main_source, late_source],
                                   'declare_bind_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r', encoding='UTF-8') as output_file:
            module_init = output_file.read().split(
                'PYBIND11_MODULE(declare_bind_py, m_)', 1)[1]

        ordered_calls = [
            'gtwrap_declare_declare_bind_py(m_);',
            'gtwrap_declare_declare_bind_late(m_);',
            'gtwrap_bind_declare_bind_py(m_);',
            'gtwrap_bind_declare_bind_late(m_);',
        ]
        positions = [module_init.index(call) for call in ordered_calls]
        self.assertEqual(positions, sorted(positions))

        with open(osp.join(self.TEST_DIR, 'pybind_wrapper.tpl'),
                  encoding='UTF-8') as template_file:
            module_template = template_file.read()
        with open(late_source, encoding='UTF-8') as interface_file:
            late_content = interface_file.read()
        wrapper = PybindWrapper(module_name='declare_bind_py',
                                top_module_namespaces=[''],
                                ignore_classes=[''],
                                module_template=module_template)
        generated_submodule = wrapper.wrap_file(
            late_content,
            module_name='declare_bind_late',
            source_name=late_source)
        self.assertIn(
            'void gtwrap_declare_declare_bind_late(py::module_ &m_)',
            generated_submodule)
        self.assertIn('void gtwrap_bind_declare_bind_late(py::module_ &m_)',
                      generated_submodule)

    def test_legacy_template_reports_missing_phase_fields(self):
        """Reject templates that cannot express module-wide two-phase init."""
        wrapper = PybindWrapper(module_name='legacy',
                                top_module_namespaces=[''],
                                ignore_classes=[''],
                                module_template=(
                                    '{module_def} {{\n'
                                    '{module_init}\n'
                                    '}}'))

        with self.assertRaisesRegex(ValueError, 'declare-then-bind fields'):
            wrapper.wrap_file('class Example {};', module_name='legacy')

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

{submodules}

{declaration_module_def} {{
{wrapped_declarations}
}}

{binding_module_def} {{
{wrapped_bindings}
}}

{module_def} {{
    m_.doc() = "pybind11 wrapper of {module_name}";
{module_init}

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
        self.assertIn(
            'static_cast<void (testing::ArgPolicyFixture::*)('
            'const testing::SpecialView&, int) const>'
            '(&testing::ArgPolicyFixture::noConvert)', content)

    def test_full_signature_callable_bindings(self):
        """Test full-signature pointer casts for ordinary callables."""
        source = osp.join(self.INTERFACE_DIR, 'class.i')
        output = self.wrap_content([source], 'class_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()

        # Const/non-const methods and static methods use explicit pointer types.
        self.assertIn(
            'static_cast<bool (Test::*)(bool) const>(&Test::return_bool)',
            content)
        self.assertIn(
            'static_cast<void (Test::*)(gtsam::Key)>(&Test::push_back)',
            content)
        self.assertIn(
            'static_cast<FunRange (*)()>(&FunRange::create)', content)

        # Python-only renaming does not require an adapter lambda.
        self.assertIn(
            '.def("lambda_",static_cast<void (Test::*)() const>'
            '(&Test::lambda)', content)
        self.assertIn(
            '.def("_repr_markdown_",static_cast<string (Test::*)('
            'const gtsam::KeyFormatter&) const>(&Test::markdown)', content)

        # Ordinary const overloads include return type and const qualification.
        self.assertIn(
            'static_cast<std::pair<gtsam::Vector,gtsam::Matrix> (Test::*)('
            'const gtsam::Vector&, const gtsam::Matrix&) const>'
            '(&Test::return_pair)', content)

        source = osp.join(self.INTERFACE_DIR, 'geometry.i')
        output = self.wrap_content([source],
                                   'geometry_py',
                                   self.PYTHON_ACTUAL_DIR,
                                   use_boost_serialization=True)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()
        self.assertIn(
            '.def_static("staticFunction",static_cast<double (*)()>'
            '(&gtsam::Point3::staticFunction))',
            content)

        source = osp.join(self.INTERFACE_DIR, 'functions.i')
        output = self.wrap_content([source], 'functions_py',
                                   self.PYTHON_ACTUAL_DIR)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()
        self.assertIn(
            'm_.def("aGlobalFunction",static_cast<gtsam::Vector (*)()>'
            '(&::aGlobalFunction))', content)
        self.assertIn(
            'static_cast<gtsam::Vector (*)(int, double)>'
            '(&::overloadedGlobalFunction)', content)

    def test_mutable_output_and_static_overloads(self):
        """Test pointer bindings for mutable output args and static overloads."""
        source = osp.join(self.INTERFACE_DIR, 'eigen_ref.i')
        output = self.wrap_content([source], 'eigen_ref_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()

        self.assertIn(
            'static_cast<gtsam::Point3 (gtsam::Pose3::*)('
            'const gtsam::Point3&, Eigen::Ref<Eigen::MatrixXd>, '
            'Eigen::Ref<Eigen::MatrixXd>) const>'
            '(&gtsam::Pose3::transformFrom)', content)
        self.assertIn(
            'static_cast<gtsam::Pose3 (*)(gtsam::Vector, '
            'Eigen::Ref<Eigen::MatrixXd>)>(&gtsam::Pose3::Expmap)', content)
        self.assertNotIn('[](', content)

    def test_member_static_name_collision(self):
        """Test member/static collisions use distinct full pointer types."""
        with open(osp.join(self.TEST_DIR, "pybind_wrapper.tpl"),
                  encoding="UTF-8") as template_file:
            module_template = template_file.read()

        wrapper = PybindWrapper(module_name='mixed_py',
                                top_module_namespaces=[''],
                                module_template=module_template)
        content = wrapper.wrap_file(
            'class Mixed { int call(int value); '
            'static int call(double value); };',
            module_name='mixed_py')

        self.assertIn(
            'static_cast<int (Mixed::*)(int)>(&Mixed::call)', content)
        self.assertIn(
            'static_cast<int (*)(double)>(&Mixed::call)', content)
        self.assertNotIn('[](', content)

    def test_lambda_adapters_are_preserved(self):
        """Test that non-forwarding and specialized bindings retain lambdas."""
        source = osp.join(self.INTERFACE_DIR, 'class.i')
        output = self.wrap_content([source], 'class_py',
                                   self.PYTHON_ACTUAL_DIR)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()

        self.assertIn(
            '.def("templatedMethodString",[](Fun<double>* self', content)
        self.assertIn('.def("print",[](Test* self)', content)
        self.assertIn('.def("__repr__",\n                    [](', content)
        self.assertIn('.def("__len__",[](FastSet* self)', content)

        source = osp.join(self.INTERFACE_DIR, 'geometry.i')
        output = self.wrap_content([source],
                                   'geometry_py',
                                   self.PYTHON_ACTUAL_DIR,
                                   use_boost_serialization=True)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()
        self.assertIn('.def("serialize", [](gtsam::Point2* self)', content)
        self.assertIn('.def(py::pickle(', content)

        source = osp.join(self.INTERFACE_DIR, 'namespaces.i')
        output = self.wrap_content([source], 'namespaces_py',
                                   self.PYTHON_ACTUAL_DIR)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()
        self.assertIn('.def("insert_vector",[](gtsam::Values* self', content)

        source = osp.join(self.INTERFACE_DIR, 'functions.i')
        output = self.wrap_content([source], 'functions_py',
                                   self.PYTHON_ACTUAL_DIR)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()
        self.assertIn(
            'm_.def("MultiTemplatedFunctionStringSize_tDouble",[](', content)

        source = osp.join(self.INTERFACE_DIR, 'special_cases.i')
        output = self.wrap_content([source], 'special_cases_py',
                                   self.PYTHON_ACTUAL_DIR)
        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()
        self.assertIn(
            '.def("addPriorPinholeCameraCal3Bundler",[](', content)

    def test_full_signature_cast_preserves_docstring(self):
        """Test that member-pointer casts retain generated docstrings."""
        with open(osp.join(self.TEST_DIR, "pybind_wrapper.tpl"),
                  encoding="UTF-8") as template_file:
            module_template = template_file.read()

        wrapper = PybindWrapper(module_name='docstring_py',
                                top_module_namespaces=[''],
                                module_template=module_template,
                                xml_source='unused')
        wrapper.xml_parser.extract_docstring = lambda *args: 'A docstring.'
        content = wrapper.wrap_file('class DocClass { int value() const; };',
                                    module_name='docstring_py')

        self.assertIn(
            '.def("value",static_cast<int (DocClass::*)() const>'
            '(&DocClass::value), "A docstring.")', content)

    def test_pybind_lambda_annotation(self):
        """Annotated adapters use the old lambda path and compile."""
        source = osp.join(self.INTERFACE_DIR, 'pybind_lambda_adapters.i')
        output = self.wrap_content(
            [source],
            'pybind_lambda_adapters_py',
            self.PYTHON_ACTUAL_DIR,
            module_template=self.MINIMAL_MODULE_TEMPLATE,
        )
        self.compare_and_diff('pybind_lambda_adapters_pybind.cpp', output)

        with open(output, 'r', encoding='UTF-8') as generated:
            content = generated.read()

        # Unannotated callables use complete function-pointer types.
        self.assertIn(
            'static_cast<int (adapters::Adapter<int>::*)(int)>'
            '(&adapters::Adapter<int>::exact)', content)
        self.assertIn(
            'static_cast<int (adapters::Adapter<int>::*)(int) const>'
            '(&adapters::Adapter<int>::exactConst)', content)
        self.assertIn(
            'static_cast<int (*)(int)>'
            '(&adapters::Adapter<int>::exactStatic)', content)
        self.assertIn(
            'static_cast<int (*)(int)>(&adapters::exactGlobal)', content)

        # Hidden and declared exact overloads are selected without annotations.
        self.assertIn(
            'static_cast<int (adapters::Adapter<int>::*)(int) const>'
            '(&adapters::Adapter<int>::hiddenOverload)', content)
        self.assertIn(
            'static_cast<int (adapters::Adapter<int>::*)(int) const>'
            '(&adapters::Adapter<int>::declaredOverload)', content)
        self.assertNotIn(
            '[](adapters::Adapter<int>* self, int value)'
            '{return self->hiddenOverload(value);}', content)
        self.assertIn(
            'static_cast<double (adapters::Adapter<int>::*)(double) const>'
            '(&adapters::Adapter<int>::declaredOverload)', content)
        self.assertIn(
            'static_cast<int (*)(int)>(&adapters::globalHidden)', content)
        self.assertIn(
            'static_cast<double (*)(double)>(&adapters::globalOverload)',
            content)

        # Intentional signature adapters retain the shared lambda implementation.
        self.assertIn(
            '.def("omittedDefault",[](adapters::Adapter<int>* self, int value)',
            content)
        self.assertIn(
            '.def_static("staticOmitted",[](int value)', content)
        self.assertIn(
            'm_adapters.def("globalOmitted",[](int value)', content)

        # Automatic template specialization, defaults, keyword renaming, and
        # return policy remain on the same lambda-emission path.
        self.assertIn(
            '.def("templatedDouble",[](adapters::Adapter<int>* self, '
            'double value)', content)
        self.assertIn(
            '.def("lambda_",[](adapters::Adapter<int>* self, '
            'const string& value) -> const auto&', content)
        self.assertIn('py::return_value_policy::reference_internal', content)
        self.assertIn(
            'gtwrap::internal::py_arg<const string&>("value") = "fallback"',
            content)

        compiler = shlex.split(os.environ.get('CXX', 'c++'))
        compiler_path = shutil.which(compiler[0])
        self.assertIsNotNone(compiler_path,
                             f"C++ compiler not found: {compiler[0]}")
        command = [
            compiler_path,
            *compiler[1:],
            '-std=c++14',
            '-fsyntax-only',
            output,
            '-I',
            osp.join(self.TEST_DIR, '..', 'pybind11', 'include'),
            '-I',
            sysconfig.get_paths()['include'],
            '-I',
            self.INTERFACE_DIR,
        ]
        result = subprocess.run(command,
                                capture_output=True,
                                text=True,
                                check=False)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_annotated_lambda_preserves_docstring(self):
        """The forced lambda path retains metadata appended by the wrapper."""
        wrapper = PybindWrapper(module_name='docstring_py',
                                top_module_namespaces=[''],
                                module_template=self.MINIMAL_MODULE_TEMPLATE,
                                xml_source='unused')
        wrapper.xml_parser.extract_docstring = lambda *args: 'An adapter.'
        content = wrapper.wrap_file(
            'class DocClass { @pybind_lambda const int& lambda('
            'int value = 1) const; };',
            module_name='docstring_py')

        self.assertIn('.def("lambda_",[](DocClass* self, int value) '
                      '-> const auto&{return self->lambda(value);}, '
                      'py::return_value_policy::reference_internal, '
                      'gtwrap::internal::py_arg<int>("value") = 1, '
                      '"An adapter.")', content)

    def test_const_ref_return_policy(self):
        """Test that methods returning const T& emit reference_internal policy.

        Without this policy, pybind11 defaults to copying the returned reference.
        With the policy, the binding keeps the reference alive via the parent object.

        Full pointer casts preserve the C++ reference return type, while
        reference_internal keeps the returned reference tied to its parent.
        """
        source = osp.join(self.INTERFACE_DIR, 'class.i')
        output = self.wrap_content([source], 'class_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r') as f:
            content = f.read()

        self.assertIn(
            'static_cast<const gtsam::Vector& (Test::*)'
            '(const gtsam::Vector&) const>(&Test::return_vector2), '
            'py::return_value_policy::reference_internal', content)
        self.assertIn(
            'static_cast<const gtsam::Matrix& (Test::*)'
            '(const gtsam::Matrix&) const>(&Test::return_matrix2), '
            'py::return_value_policy::reference_internal', content)

        # Non-ref returns (e.g. return_vector1 which returns by value) should NOT
        lines = content.split('\n')
        for line in lines:
            if 'return_vector1' in line:
                self.assertNotIn('reference_internal', line)
                self.assertIn(
                    'static_cast<gtsam::Vector (Test::*)('
                    'const gtsam::Vector&) const>', line)
            if 'return_matrix1' in line:
                self.assertNotIn('reference_internal', line)
                self.assertIn(
                    'static_cast<gtsam::Matrix (Test::*)('
                    'const gtsam::Matrix&) const>', line)

        source = osp.join(self.INTERFACE_DIR, 'return_policies.i')
        output = self.wrap_content([source], 'return_policies_py',
                                   self.PYTHON_ACTUAL_DIR)

        with open(output, 'r', encoding='UTF-8') as f:
            content = f.read()

        for line in content.split('\n'):
            if 'return_const_ref' in line:
                self.assertIn('reference_internal', line)
                self.assertIn(
                    'static_cast<const gtsam::Matrix& '
                    '(ReturnPolicyFixture::*)', line)
                self.assertNotIn('[](', line)
            if '.def("return_mutable_ref"' in line or '.def("return_value"' in line:
                self.assertNotIn('reference_internal', line)
                self.assertIn('static_cast<', line)
                self.assertNotIn('[](', line)


if __name__ == '__main__':
    unittest.main()
