import unittest
from gtsam import *
from math import *
import numpy as np


class TestCal3Unified(unittest.TestCase):

    def test_Cal3Unified(self):
        K = Cal3Unified()
        self.assertEqual(K.fx(), 1.)
        self.assertEqual(K.fx(), 1.)

if __name__ == "__main__":
    unittest.main()
