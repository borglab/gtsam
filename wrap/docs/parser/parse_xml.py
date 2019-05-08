import os
import textwrap

import os.path as path
import xml.etree.ElementTree as ET

import docs.documentation_template as template

DOXYGEN_CONF = 'conf_doxygen.py'


def generate_xml(path_to_file):
    '''Parse the file for documentation and output it as in an xml format'''
    conf_path = path.join(path.relpath(path.split(__file__)[0]), DOXYGEN_CONF)
    file_path = path.relpath(path_to_file)

    command = \
        '( cat {conf_path} ; echo "INPUT={file_path}" ) | doxygen -'.format(
            conf_path=conf_path, file_path=file_path)

    print('--------------Generating XML--------------')

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
