import os
import os.path as path
import xml.etree.ElementTree as ET

from docs.docs import ClassDoc, Doc, Docs, FreeDoc

DOXYGEN_CONF = 'doxygen.conf'


class ParseDoxygenXML():
    def __init__(self, input_path, output_path):
        """Parse the Doxygen generated XML files.

        Arguments:
        input_path -- path to the input folder or file
        output_path -- path to the output folder
        """
        self.input_path = input_path
        self.output_path = output_path

    def run(self, quiet=True):
        """Run the Doxygen XML parser.

        Arguments:
        quiet -- turn on/off the messages that are generated to standard
                 output by Doxygen (default = True)

        Returns:
            A Docs template storing all the class and free documentation in the
            file.
        """
        class_docs = {}
        free_docs = {}

        for root, dirs, files in os.walk(self.output_path):
            for f in files:
                if f.endswith('.xml'):
                    file_path = path.join(root, f)
                    tree = ET.parse(file_path)

                    if tree.getroot().tag == 'compounddef':
                        first_compound_def = tree.getroot()
                    else:
                        first_compound_def = tree.find(
                            './/{}'.format('compounddef'))

                    if first_compound_def is None:
                        continue

                    category = first_compound_def.get('kind')

                    if category == 'class':
                        class_docs[file_path] = ClassDoc(tree)

        return Docs(class_docs, free_docs)


# TODO: This should be done in the make file.
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
