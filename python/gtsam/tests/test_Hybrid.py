import gtsam
import numpy as np
from gtsam import GaussianHybridFactorGraph
from gtsam.utils.test_case import GtsamTestCase


class TestHybridElimination(GtsamTestCase):
    def setUp(self) -> None:
        self.ghfg = GaussianHybridFactorGraph()

    def test_elimination(self):
        # Check if constructed correctly
        self.assertIsInstance(self.ghfg, GaussianHybridFactorGraph)
