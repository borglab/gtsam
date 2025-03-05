import xml.etree.ElementTree as ET
import sys
import os

"""Parses and extracts docs from Doxygen-generated XML.
"""
class XMLDocParser:
    def __init__(self):
        # Memory for overloaded functions with identical parameter name sets
        self._memory = {}
        # This is useful for investigating functions that cause problems for extract_docstring.
        # Set this to true to have useful information for debugging this class, as in the CLI
        # function at the bottom of this class.
        self._verbose = False

    """Get the ElementTree of an XML file given the file name.
    If an error occurs, prints a warning and returns None.
    """
    def parse_xml(self, xml_file: str):
        try:
            return ET.parse(xml_file)
        except FileNotFoundError:
            print(f"Warning: XML file '{xml_file}' not found.")
            return None
        except ET.ParseError:
            print(f"Warning: Failed to parse XML file '{xml_file}'.")
            return None

    """Extract the docstrings for a C++ class's method from the Doxygen-generated XML.
    
    Parameters:
        xml_folder: the path to the folder that contains all of the Doxygen-generated XML.
        cpp_class: the name of the C++ class that contains the function whose docstring is to be extracted.
        cpp_method: the name of the C++ method whose docstring is to be extracted.
        method_args_names: a list of the names of the cpp_method's parameters.
    """
    def extract_docstring(self, xml_folder: str, cpp_class: str, cpp_method: str, method_args_names: list[str]):
        if self._verbose: print(f"Extracting docs for {cpp_class}.{cpp_method}")
        
        # Create the path to the Doxygen XML index file.
        xml_index_file = xml_folder + os.sep + "index.xml"

        # Parse the index file
        index_tree = self.parse_xml(xml_index_file)
        if not index_tree: 
            if self._verbose: print(f"Index file {xml_index_file} was empty.")
            return ""
        
        index_root = index_tree.getroot()

        # Find the compound with name == cpp_class
        class_index = index_root.find(f"./*[name='{cpp_class}']")

        if class_index is None:
            if self._verbose: print(f"Could not extract docs for {cpp_class}.{cpp_method}; class not found in index file.")
            return ""
        
        # Create the path to the file with the documentation for cpp_class.
        xml_class_file = xml_folder + os.sep + class_index.attrib['refid'] + '.xml'

        # Parse the class file
        class_tree = self.parse_xml(xml_class_file)
        if not class_tree:
            if self._verbose: print(f"Class file {xml_class_file} was empty.")
            return ""

        class_root = class_tree.getroot()

        # Find the member in class with name == cpp_method 
        maybe_member_defs = class_root.findall(f"compounddef/sectiondef//*[name='{cpp_method}']")

        member_defs = []

        # Optional parameters we should ignore if we encounter them in the docstring
        ignored_params = []

        # Filter out the members which don't match the method_args_names
        for maybe_member_def in maybe_member_defs:
            if self._verbose: print(f"Investigating member_def with argstring {maybe_member_def.find('argsstring').text}")
            # Find the number of required parameters and the number of total parameters from the 
            # Doxygen XML for this member_def
            params = maybe_member_def.findall("param")
            num_tot_params = len(params)
            # Calculate required params by subtracting the number of optional params (where defval is 
            # set) from the number of total params
            num_req_params = num_tot_params - sum([1 if param.find("defval") is not None else 0 for param in params])

            # If the number of parameters in method_args_names matches neither number, eliminate this member_def
            # This is done because wrap generates a python wrapper function twice for every function with
            # optional parameters: one with none of the optional parameters, and one with all of the optional
            # parameters, required.
            if len(method_args_names) != num_req_params and len(method_args_names) != num_tot_params:
                if self._verbose: print(f"Wrong number of parameters: got {len(method_args_names)}, expected required {num_req_params} or total {num_tot_params}.")
                continue

            # If the parameter names don't match, eliminate this member_def
            eliminate = False
            for i, arg_name in enumerate(method_args_names):
                # Try to find the name of the parameter in the XML
                param_name = params[i].find("declname") # declname is the tag that usually contains the param name
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
                if self._verbose: print("Names didn't match.")
                continue

            # At this point, this member_def can be assumed to be the desired function (or is indistinguishable
            # from it based on all of the reliable information we have--if this is the case, we need to rely on
            # the _memory to give the correct docs for each.)
            member_defs.append(maybe_member_def)
            if self._verbose: print("Confirmed as correct function.")

            # Remember which parameters to ignore, if any
            for i in range(len(method_args_names), num_tot_params):
                ignored_params.append(params[i].find("declname").text)

        docstring = ""

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
            function_key = f'{cpp_class}.{cpp_method}({",".join(method_args_names) if method_args_names else ""})'
            if function_key in self._memory:
                self._memory[function_key] += 1
                documenting_index = self._memory[function_key]
            else:
                self._memory[function_key] = 0

        # Extract the docs for the function that matches cpp_class.cpp_method(method_args_names).
        # If there are multiple that match, pick the first one we haven't already returned, since
        # Doxygen orders docs in the same way pybind11 orders wrapper function creation (both use
        # the order the functions are defined in the file).
        for i, member_def in enumerate(member_defs):
            # If there are multiple functions that match what we're looking for, ignore all except
            # for the one calculated by documenting_index.
            if i != documenting_index:
                continue
            
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
                    if element.tag == "para" and "parameterlist" not in [e.tag for e in element]:
                        docstring += "".join(t for t in element.itertext() if t.strip()) + " "

                # Add parameter docs
                parameter_list = detailed_description.find(".//parameterlist")
                if parameter_list is not None:
                    for i, parameter_item in enumerate(parameter_list.findall(".//parameteritem")):
                        name = parameter_item.find(".//parametername").text
                        desc = parameter_item.find(".//parameterdescription/para").text
                        if name not in ignored_params:
                            docstring += f"{name.strip() if name else f'[Parameter {i}]'}: {desc.strip() if desc else 'No description provided'}\n"

                # Add return value docs
                return_sect = detailed_description.find(".//simplesect")
                if return_sect is not None and return_sect.attrib["kind"] == "return" and return_sect.find("para").text is not None:
                    docstring += f"Returns: {return_sect.find('para').text.strip()}"

        return docstring.strip()

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python xml_parser.py <doxygen_xml_folder> <cpp_class> <cpp_method> <method_args_names (comma-separated)>")
    else:
        parser = XMLDocParser()
        parser._verbose = True
        xml_file = sys.argv[1]
        extracted_doc = parser.extract_docstring(xml_file, sys.argv[2], sys.argv[3], sys.argv[4].split(","))
        
        print()
        print(extracted_doc.strip())