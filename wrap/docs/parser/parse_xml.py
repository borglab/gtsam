import os

import docs.doc_template as template
import os.path as path
import xml.etree.ElementTree as ET

DOXYGEN_CONF = 'conf_doxygen.py'


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


def parse():
    memberdefs = find_all_elements_with_name('memberdef')

    instantiate_methods(memberdefs)


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


def instantiate_params(param_list):
    return list(map(
        lambda param:
            template.param(name=param.findtext('declname'),
                           type=param.findtext('type'),
                           default_value=param.findtext('defval')),
        param_list))


def instantiate_methods(methods):
    for method in methods:
        method_doc = template.method(
            return_type=method.findtext('type'),
            definition=method.findtext('definition'),
            argsstring=method.findtext('argsstring'),
            name=method.findtext('name'),
            reimplements=method.findtext('reimplements'),
            params=instantiate_params(method.findall('param')),
            brief_description=find_method_element_text(
                method, 'briefdescription'),
            detailed_description=find_method_element_text(
                method, 'detaileddescription'),
            inbody_description=find_method_element_text(
                method, 'inbodydescription')
        )

        if len(method_doc.params) != 0:
            param_example = '{} : {}\n'.format(
                method_doc.params[0].name, method_doc.params[0].type)
        else:
            param_example = ''

        print('Comment')
        print(textwrap.dedent('''\
            """{}\n
            {}\n
            Parameters
            ----------
            {}
            """
        ''').format(method_doc.brief_description.lstrip(),
                    method_doc.detailed_description, param_example))
