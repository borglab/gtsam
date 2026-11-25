"""Regression tests for the lightweight GTSAM source checks."""

import unittest

from check_code_quality import check_source


class CodeQualityTest(unittest.TestCase):
    def test_module_dependency_direction(self):
        self.assertEqual(
            list(check_source("gtsam/nonlinear/Foo.h", '#include <gtsam/inference/Key.h>')),
            [],
        )
        source = '#include "../nonlinear/Values.h"'
        [(line, message)] = check_source("gtsam/inference/Foo.h", source)
        self.assertEqual(line, 1)
        self.assertIn("inference cannot include nonlinear", message)

    def test_module_dependency_exception(self):
        source = '#include <gtsam/nonlinear/Values.h>'
        self.assertEqual(list(check_source("gtsam/base/numericalDerivative.h", source)), [])
        self.assertTrue(list(check_source("gtsam/base/Foo.h", source)))
        self.assertTrue(list(check_source(
            "gtsam/base/numericalDerivative.h",
            '#include <gtsam/nonlinear/NonlinearFactorGraph.h>',
        )))

    def test_module_dependency_scope(self):
        source = '#include <gtsam/slam/BetweenFactor.h>'
        for path in ("gtsam/base/tests/testFoo.cpp", "gtsam/base/base.i", "gtsam/base/wrap/Foo.h"):
            with self.subTest(path=path):
                self.assertEqual(list(check_source(path, source)), [])

    def test_geometry_in_nonlinear(self):
        source = '#include <gtsam/geometry/Pose3.h>'
        self.assertTrue(list(check_source("gtsam/nonlinear/WnoaFactor.h", source)))
        self.assertEqual(
            list(check_source("gtsam/sam/GraphvizFormatting.cpp", source)),
            [],
        )
        self.assertEqual(list(check_source("gtsam/slam/Foo.h", source)), [])

    def test_stable_dependency(self):
        source = '#include <gtsam_unstable/foo.h>\n#include "../../gtsam_unstable/bar.h"'
        self.assertEqual([line for line, _ in check_source("gtsam/base/Foo.h", source)], [1, 2])
        self.assertEqual(list(check_source("gtsam_unstable/base/Foo.h", source)), [])

    def test_wrapper_exception(self):
        path = "gtsam/navigation/navigation_geometric.i"
        self.assertEqual(
            list(check_source(path, '#include <gtsam_unstable/geometry/ABCEquivariantFilter.h>')),
            [],
        )
        self.assertTrue(list(check_source(path, '#include <gtsam_unstable/Other.h>')))

    def test_export_ownership(self):
        self.assertTrue(list(check_source("gtsam/base/Foo.h", "class GTSAM_UNSTABLE_EXPORT Foo {};")))
        self.assertTrue(list(check_source("gtsam_unstable/base/Foo.h", "class GTSAM_EXPORT Foo {};")))
        self.assertEqual(list(check_source("gtsam/base/Foo.h", "class GTSAM_EXPORT Foo {};")), [])

    def test_exception_spelling(self):
        name = "IndeterminantLinearSystemException"
        self.assertTrue(list(check_source("gtsam/linear/Foo.cpp", f"throw {name}(key);")))
        self.assertEqual(list(check_source("gtsam/linear/linearExceptions.h", f"using {name} = IndeterminateSystemException;")), [])

    def test_comments_and_literals(self):
        source = '''/*
#include <gtsam_unstable/foo.h>
GTSAM_UNSTABLE_EXPORT
*/
// IndeterminantLinearSystemException
const char* example = "GTSAM_UNSTABLE_EXPORT";
const char* raw = R"doc(
#include <gtsam_unstable/foo.h>
)doc";
'''
        self.assertEqual(list(check_source("gtsam/base/Foo.h", source)), [])


if __name__ == "__main__":
    unittest.main()
