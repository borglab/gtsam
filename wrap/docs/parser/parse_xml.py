import os

import docs.doc_template as template
import os.path as path
import xml.etree.ElementTree as ET

DOXYGEN_CONF = 'conf_doxygen.py'


def parse(input_path, output_path, quiet=False):
    '''Parse the files for documentation and store it in templates.

    Arguments:
    input_path -- path to the input folder or file
    output_path -- path to the output folder
    quiet -- turn on/off the messages that are generated to standard output by
             Doxygen

    Returns:
        A Docs template storing all the documentation in the input.
    '''
    generate_xml(input_path, output_path, quiet)

    # TODO: For each xml file -> categorize it -> move to template for its type


def generate_xml(input_path, output_path, quiet=False):
    '''Parse the file for documentation and output it as in an xml format'''
    if not quiet:
        print('--------------Generating XML--------------')

    input_path = path.relpath(input_path, os.getcwd())
    conf_path = path.relpath(path.join(path.dirname(__file__), DOXYGEN_CONF))
    output_path = path.relpath(output_path)

    if not path.isdir(output_path):
        os.mkdir(output_path)

    command = '( cat {conf_path} ; echo "INPUT={input_path}" ; echo "OUTPUT_DIRECTORY={output_path}" ; echo "EXTRACT_ALL={quiet}" ) | doxygen -'.format(
        conf_path=conf_path,
        input_path=input_path,
        output_path=output_path,
        quiet='YES' if quiet else 'NO')

    os.system(command)


def categorize_xml(tree):
    '''Determine the type of the object the xml tree represents.

    Arguments:
    tree -- an xml tree
    '''
    return find_first_element_with_tag(tree, 'compounddef').get('kind')


def find_first_element_with_tag(tree, tag):
    if tree.getroot().tag == tag:
        return tree.getroot()

    return tree.find('.//{}'.format(tag))


def find_all_elements_with_name(name):
    tree = ET.parse(
        'docs/parser/output/xml/classgtsam_1_1NoiseModelFactor1.xml')

    return tree.getroot().findall('.//{}'.format(name))


def find_method_element_text(method, name):
    element = method.find(name)

    if element is None:
        return None

    out = '' if element.text is None else element.text

    for e in list(element):
        out += e.text

    return out
