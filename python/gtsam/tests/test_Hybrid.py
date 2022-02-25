import gtsam
import numpy as np
from gtsam import GaussianHybridFactorGraph, IncrementalHybrid
from gtsam.utils.test_case import GtsamTestCase


class TestHybridElimination(GtsamTestCase):
    def setUp(self) -> None:
        self.ghfg = GaussianHybridFactorGraph()

    def test_elimination(self):
        # Check if constructed correctly
        self.assertIsInstance(self.ghfg, GaussianHybridFactorGraph)

    def test_incremental(self):
        ordering = gtsam.Ordering()
        inc = IncrementalHybrid()
        inc.update(self.ghfg, ordering)
        # inc.update(self.ghfg, ordering, 4)
