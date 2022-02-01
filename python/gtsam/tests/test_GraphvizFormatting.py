"""
See LICENSE for the license information

Unit tests for Graphviz formatting of NonlinearFactorGraph.
Author: senselessDev (contact by mentioning on GitHub, e.g. in PR#1059)
"""

# pylint: disable=no-member, invalid-name

import unittest
import textwrap

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestGraphvizFormatting(GtsamTestCase):
    """Tests for saving NonlinearFactorGraph to GraphViz format."""

    def setUp(self):
        self.graph = gtsam.NonlinearFactorGraph()

        odometry = gtsam.Pose2(2.0, 0.0, 0.0)
        odometryNoise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.2, 0.2, 0.1]))
        self.graph.add(gtsam.BetweenFactorPose2(0, 1, odometry, odometryNoise))
        self.graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometryNoise))

        self.values = gtsam.Values()
        self.values.insert_pose2(0, gtsam.Pose2(0., 0., 0.))
        self.values.insert_pose2(1, gtsam.Pose2(2., 0., 0.))
        self.values.insert_pose2(2, gtsam.Pose2(4., 0., 0.))

    def test_default(self):
        """Test with default GraphvizFormatting"""
        expected_result = """\
            graph {
              size="5,5";

              var0[label="0", pos="0,0!"];
              var1[label="1", pos="0,2!"];
              var2[label="2", pos="0,4!"];

              factor0[label="", shape=point];
              var0--factor0;
              var1--factor0;
              factor1[label="", shape=point];
              var1--factor1;
              var2--factor1;
            }
        """

        self.assertEqual(self.graph.dot(self.values),
                         textwrap.dedent(expected_result))

    def test_swapped_axes(self):
        """Test with user-defined GraphvizFormatting swapping x and y"""
        expected_result = """\
            graph {
              size="5,5";

              var0[label="0", pos="0,0!"];
              var1[label="1", pos="2,0!"];
              var2[label="2", pos="4,0!"];

              factor0[label="", shape=point];
              var0--factor0;
              var1--factor0;
              factor1[label="", shape=point];
              var1--factor1;
              var2--factor1;
            }
        """

        graphviz_formatting = gtsam.GraphvizFormatting()
        graphviz_formatting.paperHorizontalAxis = gtsam.GraphvizFormatting.Axis.X
        graphviz_formatting.paperVerticalAxis = gtsam.GraphvizFormatting.Axis.Y
        self.assertEqual(self.graph.dot(self.values,
                                        writer=graphviz_formatting),
                         textwrap.dedent(expected_result))

    def test_factor_points(self):
        """Test with user-defined GraphvizFormatting without factor points"""
        expected_result = """\
            graph {
              size="5,5";

              var0[label="0", pos="0,0!"];
              var1[label="1", pos="0,2!"];
              var2[label="2", pos="0,4!"];

              var0--var1;
              var1--var2;
            }
        """

        graphviz_formatting = gtsam.GraphvizFormatting()
        graphviz_formatting.plotFactorPoints = False

        self.assertEqual(self.graph.dot(self.values,
                                        writer=graphviz_formatting),
                         textwrap.dedent(expected_result))

    def test_width_height(self):
        """Test with user-defined GraphvizFormatting for width and height"""
        expected_result = """\
            graph {
              size="20,10";

              var0[label="0", pos="0,0!"];
              var1[label="1", pos="0,2!"];
              var2[label="2", pos="0,4!"];

              factor0[label="", shape=point];
              var0--factor0;
              var1--factor0;
              factor1[label="", shape=point];
              var1--factor1;
              var2--factor1;
            }
        """

        graphviz_formatting = gtsam.GraphvizFormatting()
        graphviz_formatting.figureWidthInches = 20
        graphviz_formatting.figureHeightInches = 10

        self.assertEqual(self.graph.dot(self.values,
                                        writer=graphviz_formatting),
                         textwrap.dedent(expected_result))


if __name__ == "__main__":
    unittest.main()
