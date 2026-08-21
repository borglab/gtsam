"""Python wrapper tests for the public CPU SFM optimizer configuration."""

import unittest

import gtsam


class TestSfmLevenbergMarquardt(unittest.TestCase):

    def test_cpu_defaults_select_fast_path(self):
        params = gtsam.SfmLevenbergMarquardtParams.ceresDefaults()
        solver = (
            gtsam.NonlinearOptimizerParams.LinearSolverType
            .MULTIFRONTAL_SOLVER
        )
        self.assertEqual(gtsam.SfmEliminationMode.Full,
                         params.getEliminationMode())
        self.assertEqual(solver, params.getLinearSolver())

    def test_typed_configuration_round_trip(self):
        params = gtsam.SfmLevenbergMarquardtParams.ceresDefaults()
        params.setEliminationMode(gtsam.SfmEliminationMode.Schur)
        solver = (
            gtsam.NonlinearOptimizerParams.LinearSolverType
            .MULTIFRONTAL_SOLVER
        )
        params.setLinearSolver(solver)

        self.assertEqual(gtsam.SfmEliminationMode.Schur,
                         params.getEliminationMode())
        self.assertEqual(solver, params.getLinearSolver())
        self.assertEqual("MULTIFRONTAL_SOLVER",
                         params.getLinearSolverType())

    def test_string_solver_api_remains_compatible(self):
        params = gtsam.SfmLevenbergMarquardtParams()
        params.setLinearSolverType("SEQUENTIAL_QR")
        expected = (
            gtsam.NonlinearOptimizerParams.LinearSolverType.SEQUENTIAL_QR
        )
        self.assertEqual(expected, params.getLinearSolver())


if __name__ == "__main__":
    unittest.main()
