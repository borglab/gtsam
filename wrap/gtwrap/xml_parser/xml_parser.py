import os
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


class XMLDocParser:
    """
    Parses and extracts docs from Doxygen-generated XML.
    """

    def __init__(self):
        # Memory for overloaded functions with identical parameter name sets
        self._memory = {}
        # This is useful for investigating functions that cause problems for extract_docstring.
        # Set this to true to have useful information for debugging this class, as in the CLI
        # function at the bottom of this class.
        self._verbose = False

    def parse_xml(self, xml_file: str):
        """
        Get the ElementTree of an XML file given the file name.
        If an error occurs, prints a warning and returns None.
        """
        try:
            return ET.parse(xml_file)
        except FileNotFoundError:
            print(f"Warning: XML file '{xml_file}' not found.")
            return None
        except ET.ParseError:
            print(f"Warning: Failed to parse XML file '{xml_file}'.")
            return None

    def extract_docstring(self, xml_folder: str, cpp_class: str,
                          cpp_method: str, method_args_names: 'list[str]'):
        """
        Extract the docstrings for a C++ class's method from the Doxygen-generated XML.
    
        Args:
            xml_folder (str): The path to the folder that contains all of the Doxygen-generated XML.
            cpp_class (str): The name of the C++ class that contains the function whose docstring is to be extracted.
            cpp_method (str): The name of the C++ method whose docstring is to be extracted.
            method_args_names (list): A list of the names of the cpp_method's parameters.
        """
        self.print_if_verbose(f"Extracting docs for {cpp_class}.{cpp_method}")

        # Get all of the member definitions in cpp_class with name cpp_method
        maybe_member_defs = self.get_member_defs(xml_folder, cpp_class,
                                                 cpp_method)

        # Filter member definitions which don't match the given argument names
        member_defs, ignored_params = self.filter_member_defs(
            maybe_member_defs, method_args_names)

        # Find which member to get docs from, if there are multiple that match in name and args
        documenting_index = self.determine_documenting_index(
            cpp_class, cpp_method, method_args_names, member_defs)

        # Extract the docs for the function that matches cpp_class.cpp_method(*method_args_names).
        return self.get_formatted_docstring(member_defs[documenting_index],
                                            ignored_params)

    def get_member_defs(self, xml_folder: str, cpp_class: str,
                        cpp_method: str):
        """Get all of the member definitions in cpp_class with name cpp_method.

        Args:
            xml_folder (str): The folder containing the Doxygen XML documentation.
            cpp_class (str): The name of the C++ class that contains the function whose docstring is to be extracted.
            cpp_method (str): The name of the C++ method whose docstring is to be extracted.

        Returns:
            list: All of the member definitions in cpp_class with name cpp_method.
        """
        xml_folder_path = Path(xml_folder)

        # Create the path to the Doxygen XML index file.
        xml_index_file = xml_folder_path / "index.xml"

        # Parse the index file
        index_tree = self.parse_xml(xml_index_file)
        if not index_tree:
            self.print_if_verbose(f"Index file {xml_index_file} was empty.")
            return ""

        index_root = index_tree.getroot()

        # Find the compound with name == cpp_class
        class_index = index_root.find(f"./*[name='{cpp_class}']")

        if class_index is None:
            self.print_if_verbose(
                f"Could not extract docs for {cpp_class}.{cpp_method}; class not found in index file."
            )
            return ""

        # Create the path to the file with the documentation for cpp_class.
        xml_class_file = xml_folder_path / class_index.attrib['refid'] + '.xml'

        # Parse the class file
        class_tree = self.parse_xml(xml_class_file)
        if not class_tree:
            self.print_if_verbose(f"Class file {xml_class_file} was empty.")
            return ""

        class_root = class_tree.getroot()

        # Find the member(s) in cpp_class with name == cpp_method
        maybe_member_defs = class_root.findall(
            f"compounddef/sectiondef//*[name='{cpp_method}']")

        return maybe_member_defs

    def filter_member_defs(self, maybe_member_defs: list,
                           method_args_names: list):
        """
        Remove member definitions which do not match the supplied argument names list.

        Args:
            maybe_member_defs (list): The list of all member definitions in the class which share the same name.
            method_args_names (list): The list of argument names in the definition of the function whose documentation is desired.
                Supplying the argument names allows for the filtering of overloaded functions with the same name but different arguments.

        Returns:
            tuple[list, list]: (the filtered member definitions, parameters which should be ignored because they are optional)
        """
        member_defs = []

        # Optional parameters we should ignore if we encounter them in the docstring
        ignored_params = []

        # Filter out the members which don't match the method_args_names
        for maybe_member_def in maybe_member_defs:
            self.print_if_verbose(
                f"Investigating member_def with argstring {maybe_member_def.find('argsstring').text}"
            )
            # Find the number of required parameters and the number of total parameters from the
            # Doxygen XML for this member_def
            params = maybe_member_def.findall("param")
            num_tot_params = len(params)
            # Calculate required params by subtracting the number of optional params (params where defval is
            # set--defval means default value) from the number of total params
            num_req_params = num_tot_params - sum([
                1 if param.find("defval") is not None else 0
                for param in params
            ])

            # If the number of parameters in method_args_names matches neither number, eliminate this member_def
            # This is done because wrap generates a python wrapper function twice for every function with
            # optional parameters: one with none of the optional parameters, and one with all of the optional
            # parameters, required.
            if len(method_args_names) != num_req_params and len(
                    method_args_names) != num_tot_params:
                self.print_if_verbose(
                    f"Wrong number of parameters: got {len(method_args_names)}, expected required {num_req_params} or total {num_tot_params}."
                )
                continue

            # If the parameter names don't match, eliminate this member_def
            eliminate = False
            for i, arg_name in enumerate(method_args_names):
                # Try to find the name of the parameter in the XML
                param_name = params[i].find(
                    "declname"
                )  # declname is the tag that usually contains the param name
                # If we couldn't find the declname, try the defname (used uncommonly)
                if param_name is None:
                    param_name = params[i].find("defname")
                if param_name is None:
                    # Can't find the name for this parameter. This may be an unreachable statement but Doxygen is
                    # not well-documented enough to rely on a <declname> or a <defname> always being defined inside a <param>.
                    eliminate = True
                    continue
                # Eliminate if any param name doesn't match the expected name
                if arg_name != param_name.text:
                    eliminate = True
            if eliminate:
                self.print_if_verbose("Names didn't match.")
                continue

            # At this point, this member_def can be assumed to be the desired function (or is indistinguishable
            # from it based on all of the reliable information we have--if this is the case, we need to rely on
            # the _memory to give the correct docs for each.)
            member_defs.append(maybe_member_def)
            self.print_if_verbose("Confirmed as correct function.")

            # Remember which parameters to ignore, if any
            for i in range(len(method_args_names), num_tot_params):
                ignored_params.append(params[i].find("declname").text)

        return member_defs, ignored_params

    def determine_documenting_index(self, cpp_class: str, cpp_method: str,
                                    method_args_names: list,
                                    member_defs: list):
        """
        Determine which member definition to retrieve documentation from, if there are multiple.

        Args:
            cpp_class (str): The name of the C++ class that contains the function whose docstring is to be extracted.
            cpp_method (str): The name of the C++ method whose docstring is to be extracted.
            method_args_names (list): A list of the names of the cpp_method's parameters.
            member_defs (list): All of the member definitions of cpp_class which match cpp_method in name 
                and whose arguments have the same names as method_args_names.

        Returns:
            int: The index indicating which member definition to document.
        """
        # If there are multiple member defs that match the method args names,
        # remember how many we've encountered already so that we can return
        # the docs for the first one we haven't yet extracted.
        # This is only relevant if there are overloaded functions where the
        # parameter types are different but the parameter names are the same,
        # e.g. foo(int bar) and foo(string bar). The parameter types cannot be
        # relied on because they cannot be assumed to be the same between GTSAM
        # implementation and pybind11 generated wrapper, e.g. OptionalJacobian
        # in GTSAM becomes Eigen::Matrix in the pybind11 code.
        documenting_index = 0
        if len(member_defs) > 1:
            function_key = f"{cpp_class}.{cpp_method}({','.join(method_args_names) if method_args_names else ''})"
            if function_key in self._memory:
                self._memory[function_key] += 1
                documenting_index = self._memory[function_key]
            else:
                self._memory[function_key] = 0

        return documenting_index

    def get_formatted_docstring(self,
                                member_def: 'xml.etree.ElementTree.Element',
                                ignored_params: list):
        """Gets the formatted docstring for the supplied XML element representing a member definition.

        Args:
            member_def (xml.etree.ElementTree.Element): The member definition to document.
            ignored_params (list): The optional parameters which should be ignored, if any.

        Returns:
            str: The formatted docstring.
        """
        docstring = ""

        brief_description = member_def.find(".//briefdescription")
        detailed_description = member_def.find(".//detaileddescription")

        # Add the brief description first, if it exists.
        if brief_description is not None:
            for para in brief_description.findall("para"):
                docstring += "".join(t for t in para.itertext() if t.strip())

        # Add the detailed description. This includes the parameter list and the return value.
        if detailed_description is not None:
            docstring += "\n"
            # Add non-parameter detailed description
            for element in list(detailed_description):
                if element.tag == "para" and "parameterlist" not in [
                        e.tag for e in element
                ]:
                    docstring += "".join(
                        t for t in element.itertext() if t.strip()) + " "

            # Add parameter docs
            parameter_list = detailed_description.find(".//parameterlist")
            if parameter_list is not None:
                for i, parameter_item in enumerate(
                        parameter_list.findall(".//parameteritem")):
                    name = parameter_item.find(".//parametername").text
                    desc = parameter_item.find(
                        ".//parameterdescription/para").text
                    if name not in ignored_params:
                        docstring += f"{name.strip() if name else f'[Parameter {i}]'}: {desc.strip() if desc else 'No description provided'}\n"

            # Add return value docs
            return_sect = detailed_description.find(".//simplesect")
            if return_sect is not None and return_sect.attrib[
                    "kind"] == "return" and return_sect.find(
                        "para").text is not None:
                docstring += f"Returns: {return_sect.find('para').text.strip()}"

        return docstring.strip()

    def print_if_verbose(self, text: str):
        """
        Print text if the parser is in verbose mode.
        """
        if self._verbose:
            print(text)


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(
            "Usage: python xml_parser.py <doxygen_xml_folder> <cpp_class> <cpp_method> <method_args_names (comma-separated)>"
        )
    else:
        parser = XMLDocParser()
        parser._verbose = True
        xml_file = sys.argv[1]
        extracted_doc = parser.extract_docstring(xml_file, sys.argv[2],
                                                 sys.argv[3],
                                                 sys.argv[4].split(","))

        print()
        print(extracted_doc.strip())
