import os

from docs.doc_template import ClassDoc, Doc, Docs, FreeDoc
import os.path as path
import xml.etree.ElementTree as ET

DOXYGEN_CONF = 'doxygen.conf'


def parse(input_path, output_path, quiet=False, generate_xml_flag=True):
    '''Parse the files for documentation and store it in templates.

    Arguments:
    input_path -- path to the input folder or file
    output_path -- path to the output folder
    quiet -- turn on/off the messages that are generated to standard output by
             Doxygen (default = False)
    generate_xml -- use Doxygen to generate xml (default = True)

    Returns:
        A Docs template storing all the documentation in the input.
    '''
    if generate_xml_flag:
        generate_xml(input_path, output_path, quiet)

    class_docs = {}
    free_docs = {}

    for root, dirs, files in os.walk(output_path):
        for f in files:
            if f.endswith('.xml'):
                file_path = path.join(root, f)
                doc = init_doc(file_path)

                if isinstance(doc, ClassDoc):
                    class_docs[file_path] = doc

    return Docs(class_docs, free_docs)


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
        quiet='YES' if quiet else 'NO'
    )

    os.system(command)


def categorize_xml(tree):
    '''Determine the type of the object the xml tree represents.

    Arguments:
    tree -- an xml tree or path to xml string
    '''
    if isinstance(tree, str):
        tree = ET.parse(tree)

    first_compound_def = find_first_element_with_tag(tree, 'compounddef')

    if first_compound_def is None:
        return first_compound_def

    return first_compound_def.get('kind')


def init_doc(file_path):
    '''Initialize documentation given its type.

    Categorize the xml tree at file_path and initiliaze the corresponding doc
    type with the xml tree and other relevant information.

    Arguments:
    file_path -- path to the xml tree

    Returns:
        An initialized Doc
    '''
    tree = ET.parse(file_path)

    category = categorize_xml(tree)

    if category == 'class':
        return ClassDoc(tree)


def find_first_element_with_tag(tree, tag):
    if tree.getroot().tag == tag:
        return tree.getroot()

    return tree.find('.//{}'.format(tag))


def find_all_elements(tree, name, tag):
    return tree.find('.//{}'.format(tag))


def find_method_element_text(method, name):
    element = method.find(name)

    if element is None:
        return None

    out = '' if element.text is None else element.text

    for e in list(element):
        out += e.text

    return out
