"""
Unit test for documentation generation
Author: Matthew Sklar
Date: May 2019
"""
import filecmp
import os
import sys
import shutil
import unittest

import os.path as path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import docs.parser.parse_xml as parser


class TestDocument(unittest.TestCase):
    DIR_NAME = path.dirname(__file__)

    DOC_DIR = 'doc-test-files'
    OUTPUT_XML_DIR = 'actual-xml-generation'
    EXPECTED_XML_DIR = 'expected-xml-generation'

    DOC_DIR_PATH = path.abspath(path.join(DIR_NAME, DOC_DIR))
    OUTPUT_XML_DIR_PATH = path.abspath(path.join(DIR_NAME, OUTPUT_XML_DIR))
    EXPECTED_XML_DIR_PATH = path.abspath(path.join(DIR_NAME, EXPECTED_XML_DIR))

    def test_xml_generation(self):
        shutil.rmtree(self.OUTPUT_XML_DIR_PATH, ignore_errors=True)
        parser.generate_xml(
            self.DOC_DIR_PATH, self.OUTPUT_XML_DIR_PATH, quiet=True)

        self.assertTrue(os.path.isdir(self.OUTPUT_XML_DIR_PATH))

        shutil.rmtree(path.join(self.OUTPUT_XML_DIR_PATH, 'xml'))
        parser.generate_xml(
            self.DOC_DIR_PATH, self.OUTPUT_XML_DIR_PATH, quiet=True)

        dircmp = filecmp.dircmp(
            self.OUTPUT_XML_DIR_PATH, self.EXPECTED_XML_DIR_PATH)

        self.assertTrue(not dircmp.diff_files and not dircmp.funny_files)

    def test_documentation_python(self):
        """Check generation of python documentation"""
        for root, dirs, files in os.walk(self.DOC_DIR_PATH):
            for f in files:
                if f.endswith('.h'):
                    print(f)


if __name__ == "__main__":
    unittest.main()
