import unittest
from gtsam import *

#https://docs.python.org/2/library/unittest.html
class TestPoint2(unittest.TestCase):
    def setUp(self):
        self.point = Point2()

    def test_constructor(self):
        pass

if __name__ == '__main__':
    unittest.main()
