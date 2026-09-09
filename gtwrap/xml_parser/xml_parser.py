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
        # Verbosity is useful for investigating functions that cause problems for extract_docstring.
        # Set this to true to have useful information for debugging this class, as in the CLI
        # function at the bottom of this class.
        self._verbose = False
        # Cache parsed XML trees to avoid re-parsing during recursive calls
        self._parsed_xml_cache = {}

    def _get_class_xml_path(self, xml_folder: str,
                            cpp_class: str) -> Path | None:
        """Finds the XML file path for a given class name using the index."""
        xml_folder_path = Path(xml_folder)
        xml_index_file = xml_folder_path / "index.xml"

        index_tree = self.parse_xml(xml_index_file)
        if not index_tree:
            self.print_if_verbose(
                f"Index file {xml_index_file} was empty or failed to parse.")
            return None

        index_root = index_tree.getroot()
        class_index = index_root.find(
            f".//compound[@kind='class'][name='{cpp_class}']")
        if class_index is None:
            # Also check for structs
            class_index = index_root.find(
                f".//compound[@kind='struct'][name='{cpp_class}']")

        if class_index is None:
            self.print_if_verbose(
                f"Class or Struct '{cpp_class}' not found in index file {xml_index_file}."
            )
            return None

        refid = class_index.attrib.get('refid')
        if not refid:
            self.print_if_verbose(
                f"Class or Struct '{cpp_class}' found in index, but has no refid."
            )
            return None

        return xml_folder_path / f"{refid}.xml"

    def parse_xml(self, xml_file: Path | str):
        """
        Get the ElementTree of an XML file given the file name.
        Uses a cache to avoid re-parsing.
        If an error occurs, prints a warning and returns None.
        """
        xml_file_path = Path(xml_file)
        file_key = str(xml_file_path.resolve())

        if file_key in self._parsed_xml_cache:
            return self._parsed_xml_cache[file_key]

        try:
            tree = ET.parse(xml_file_path)
            self._parsed_xml_cache[file_key] = tree
            return tree
        except FileNotFoundError:
            print(f"Warning: XML file '{xml_file_path}' not found.")
            self._parsed_xml_cache[file_key] = None  # Cache failure
            return None
        except ET.ParseError:
            print(f"Warning: Failed to parse XML file '{xml_file_path}'.")
            self._parsed_xml_cache[file_key] = None  # Cache failure
            return None

    def extract_docstring(self, xml_folder: str, cpp_class: str,
                          cpp_method: str, method_args_names: 'list[str]'):
        """
        Extract the docstrings for a C++ class's method from the Doxygen-generated XML.
        If not found in the specified class, searches parent classes recursively.

        Args:
            xml_folder (str): The path to the folder that contains all of the Doxygen-generated XML.
            cpp_class (str): The name of the C++ class that contains the function whose docstring is to be extracted.
            cpp_method (str): The name of the C++ method whose docstring is to be extracted.
            method_args_names (list): A list of the names of the cpp_method's parameters.
        """
        self.print_if_verbose(
            f"--- Attempting to extract docs for {cpp_class}.{cpp_method} ---")

        # Find the XML file for the current class
        xml_class_file = self._get_class_xml_path(xml_folder, cpp_class)
        if not xml_class_file:
            self.print_if_verbose(
                f"Could not find XML file for class {cpp_class}.")
            return ""  # Cannot proceed without the class file

        # Parse the class XML file
        class_tree = self.parse_xml(xml_class_file)
        if not class_tree:
            self.print_if_verbose(
                f"Class file {xml_class_file} was empty or failed to parse.")
            return ""

        class_root = class_tree.getroot()

        # --- Step 1: Search for the method definition directly within this class ---
        maybe_member_defs = self.get_member_defs_from_root(
            class_root, cpp_method)

        # Filter member definitions which don't match the given argument names
        member_defs, ignored_params = self.filter_member_defs(
            maybe_member_defs, method_args_names, cpp_class,
            cpp_method)  # Pass class/method for verbose printing

        # If we found matching definitions in *this* class
        if member_defs:
            # Find which member to get docs from, if there are multiple that match in name and args
            documenting_index = self.determine_documenting_index(
                cpp_class, cpp_method, method_args_names, member_defs)

            # Ensure the index is valid (can happen with memory issues or complex inheritance/overload combos)
            if documenting_index < len(member_defs):
                self.print_if_verbose(
                    f"Found direct documentation for {cpp_class}.{cpp_method}."
                )
                # Extract the docs for the function that matches cpp_class.cpp_method(*method_args_names).
                return self.get_formatted_docstring(
                    member_defs[documenting_index], ignored_params)
            else:
                self.print_if_verbose(
                    f"Calculated documenting_index {documenting_index} is out of bounds for {cpp_class}.{cpp_method} (len={len(member_defs)})."
                )
                # Fall through to parent search, maybe the specific overload documented is in parent

        # --- Step 2: If not found directly, search in parent classes ---
        self.print_if_verbose(
            f"No direct documentation found for {cpp_class}.{cpp_method} with matching args. Checking base classes..."
        )

        # Find base classes from the current class's XML
        compound_def = class_root.find("compounddef")
        if compound_def is None:
            self.print_if_verbose(
                f"Could not find <compounddef> in {xml_class_file}")
            return ""

        base_refs = compound_def.findall("basecompoundref")
        for base_ref in base_refs:
            base_refid = base_ref.attrib.get("refid")
            if not base_refid:
                continue

            # We need the *name* of the base class to perform the recursive search correctly.
            # Doxygen often uses the refid as the filename.
            parent_xml_file = Path(xml_folder) / f"{base_refid}.xml"
            parent_tree = self.parse_xml(parent_xml_file)
            if not parent_tree:
                self.print_if_verbose(
                    f"Could not parse parent XML file {parent_xml_file} for refid {base_refid}."
                )
                continue

            parent_root = parent_tree.getroot()
            parent_compound_def = parent_root.find("compounddef")
            if parent_compound_def is None:
                self.print_if_verbose(
                    f"Could not find <compounddef> in parent XML {parent_xml_file}."
                )
                continue

            parent_name_element = parent_compound_def.find("compoundname")
            if parent_name_element is None or not parent_name_element.text:
                self.print_if_verbose(
                    f"Could not find <compoundname> in parent XML {parent_xml_file}."
                )
                continue

            parent_class_name = parent_name_element.text
            self.print_if_verbose(
                f"Recursively searching for {cpp_method} in base class: {parent_class_name} (refid: {base_refid})"
            )

            # Recursive call for the parent class
            # Reset memory for the parent context? No, memory should track specific overloads globally.
            parent_docstring = self.extract_docstring(xml_folder,
                                                      parent_class_name,
                                                      cpp_method,
                                                      method_args_names)

            if parent_docstring:
                self.print_if_verbose(
                    f"Found documentation for {cpp_method} in base class {parent_class_name}."
                )
                # NOTE: We return the docstring found in the parent. The _memory tracking in
                # determine_documenting_index handles cases where the *same* overload
                # signature (name+arg_names) appears multiple times across the hierarchy,
                # ensuring we pick the 'next' available one based on previous calls.
                return parent_docstring
            else:
                self.print_if_verbose(
                    f"Method {cpp_method} not found or documented in base class {parent_class_name}."
                )

        # --- Step 3: If not found anywhere ---
        self.print_if_verbose(
            f"Method {cpp_method} with matching args not documented in {cpp_class} or any base classes."
        )
        return ""

    def get_member_defs_from_root(self, class_root: ET.Element,
                                  cpp_method: str) -> list[ET.Element]:
        """Finds member definitions for a method name within a given class XML root."""
        # Find the member(s) in cpp_class with name == cpp_method
        # Search within <sectiondef> elements which contain function definitions
        member_defs = class_root.findall(
            f"compounddef/sectiondef[@kind='public-func']/memberdef[@kind='function'][name='{cpp_method}']"
        )
        member_defs.extend(
            class_root.findall(
                f"compounddef/sectiondef[@kind='public-static-func']/memberdef[@kind='function'][name='{cpp_method}']"
            ))
        # Add other kinds if necessary (e.g., protected-func, user-defined sections containing functions)
        member_defs.extend(
            class_root.findall(
                f"compounddef/sectiondef[@kind='user-defined']/memberdef[@kind='function'][name='{cpp_method}']"
            ))
        # Search for typedefs too, if methods might be hidden behind them. Unlikely for funcs.

        return member_defs

    def filter_member_defs(self, maybe_member_defs: list[ET.Element],
                           method_args_names: list[str], cpp_class: str,
                           cpp_method: str):
        """
        Remove member definitions which do not match the supplied argument names list.

        Args:
            maybe_member_defs (list): The list of all member definitions in the class which share the same name.
            method_args_names (list): The list of argument names in the definition of the function whose documentation is desired.
            cpp_class (str): The name of the class being investigated (for verbose output).
            cpp_method (str): The name of the method being investigated (for verbose output).

        Returns:
            tuple[list, list]: (the filtered member definitions, parameters which should be ignored because they are optional)
        """
        member_defs = []
        # Track ignored params per specific successful match
        ignored_params = []

        # Filter out the members which don't match the method_args_names
        for maybe_member_def in maybe_member_defs:
            # Ignored params for *this specific* potential match
            current_ignored = []
            args_string_elem = maybe_member_def.find('argsstring')
            args_string_text = args_string_elem.text if args_string_elem is not None else "[no argsstring]"
            self.print_if_verbose(
                f"  Investigating potential match for {cpp_class}.{cpp_method}: "
                f"argstring '{args_string_text}' (loc: {maybe_member_def.find('location').attrib.get('file', '?')}:{maybe_member_def.find('location').attrib.get('line', '?')})"
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

            self.print_if_verbose(
                f"    XML Params: Total={num_tot_params}, Required={num_req_params}. Provided args count: {len(method_args_names)}"
            )

            # If the number of parameters in method_args_names matches neither number, eliminate this member_def
            # This is done because wrap generates a python wrapper function twice for every function with
            # optional parameters: one with none of the optional parameters, and one with all of the optional
            # parameters, required.
            if len(method_args_names) != num_req_params and len(
                    method_args_names) != num_tot_params:
                self.print_if_verbose(
                    f"    Parameter count mismatch. Skipping.")
                continue

            eliminate = False

            # If the parameter names don't match, eliminate this member_def
            # Ensure we don't try to access params beyond the XML list size
            if len(method_args_names) > num_tot_params:
                eliminate = True
                self.print_if_verbose(
                    f"    Provided args count ({len(method_args_names)}) > XML total params ({num_tot_params}). Skipping."
                )
            else:
                for i, arg_name in enumerate(method_args_names):
                    # Try to find the name of the parameter in the XML
                    param_elem = params[i]
                    # declname is the tag that usually contains the param name
                    param_name_elem = param_elem.find("declname")
                    # If we couldn't find the declname, try the defname (used uncommonly)
                    if param_name_elem is None:
                        param_name_elem = param_elem.find("defname")

                    if param_name_elem is None or param_name_elem.text is None:
                        # Can't find the name for this parameter. This may be an unreachable statement but
                        # I don't want to rely on a <declname> or a <defname> always being defined inside a <param>.
                        self.print_if_verbose(
                            f"    Could not find XML name for parameter index {i}. Skipping."
                        )
                        eliminate = True
                        break  # No point checking further params for this def

                    xml_param_name = param_name_elem.text
                    # Eliminate if any param name doesn't match the expected name
                    if arg_name != xml_param_name:
                        self.print_if_verbose(
                            f"    Parameter name mismatch at index {i}: Provided='{arg_name}', XML='{xml_param_name}'. Skipping."
                        )
                        eliminate = True
                        break  # No point checking further params for this def

            if eliminate:
                continue

            # At this point, this member_def matches the required/total parameter count AND the names match up to method_args_names.
            # This is a candidate function.
            member_defs.append(maybe_member_def)
            self.print_if_verbose(
                "    Confirmed as candidate function by arg names and count.")

            # If this matched based on the *total* number of parameters, there are no ignored optional parameters
            # for *this specific call*. If it matched based on the *required* number, then the remaining
            # parameters in the XML are the ones ignored *for this specific call*.
            if len(method_args_names
                   ) == num_req_params and num_req_params != num_tot_params:
                self.print_if_verbose(
                    f"    Matched on required params ({num_req_params}). Identifying ignored optional params..."
                )
                for i in range(num_req_params, num_tot_params):
                    ignored_name_elem = params[i].find("declname")
                    if ignored_name_elem is None:
                        ignored_name_elem = params[i].find("defname")
                    if ignored_name_elem is not None and ignored_name_elem.text:
                        current_ignored.append(ignored_name_elem.text)
                        self.print_if_verbose(
                            f"      Ignoring optional param: {ignored_name_elem.text}"
                        )
                # The ignored_params list will contain those
                # from the *last* successful match. This relies on determine_documenting_index
                # picking correctly. A cleaner way would be to associate ignored_params *with* the member_def.
                ignored_params = current_ignored  # Overwrite with ignored params from this match
            else:
                self.print_if_verbose(
                    f"    Matched on total params ({num_tot_params}) or req==tot. No ignored optional params for this match."
                )
                # No ignored params if we matched the total count
                ignored_params = []

        return member_defs, ignored_params  # Return the list associated with the last match found

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
        documenting_index = 0
        num_matches = len(member_defs)

        if num_matches <= 1:
            return 0  # Trivial case

        # If there are multiple member defs that survived filtering (e.g. same name, same arg *names*,
        # but potentially different types, or defined in both parent and child),
        # use memory to cycle through them.
        function_key = f"{cpp_class}.{cpp_method}({','.join(method_args_names) if method_args_names else ''})"

        # How many times have we *previously* returned a doc for this exact signature?
        # Use -1 to start index at 0 after increment
        times_documented = self._memory.get(function_key, -1)

        # The index we should use *this time* is the number of times documented previously.
        documenting_index = times_documented + 1

        # Make sure the index is within the bounds of the *currently found* matches.
        # If we've documented more times than there are current matches (e.g., due to complex
        # inheritance/overload scenarios or issues in filtering), default to 0.
        if documenting_index >= num_matches:
            self.print_if_verbose(
                f"    Memory index {documenting_index} >= num_matches {num_matches} for {function_key}. Wrapping to 0."
            )
            documenting_index = 0
            self._memory[function_key] = 0  # Store the index we are returning
        else:
            # Store the index we are about to return, so the *next* call gets the subsequent one.
            self._memory[function_key] = documenting_index

        self.print_if_verbose(
            f"  Multiple matches ({num_matches}) found for {function_key}. Using index {documenting_index} based on memory."
        )
        return documenting_index

    def get_formatted_docstring(self, member_def: 'ET.Element',
                                ignored_params: list):
        """Gets the formatted docstring for the supplied XML element representing a member definition.

        Args:
            member_def (xml.etree.ElementTree.Element): The member definition to document.
            ignored_params (list): The optional parameters which should be ignored *for this specific overload match*, if any.

        Returns:
            str: The formatted docstring. Returns empty string if member_def lacks documentation tags.
        """
        docstring = ""
        has_content = False

        # Location info for debugging which definition is being used
        location_elem = member_def.find('location')
        loc_info = f"(from {location_elem.attrib.get('file', '?')}:{location_elem.attrib.get('line', '?')})" if location_elem is not None else ""
        self.print_if_verbose(f"  Formatting docstring {loc_info}")

        brief_description = member_def.find("./briefdescription")
        detailed_description = member_def.find("./detaileddescription")

        # Add the brief description first, if it exists and has content.
        if brief_description is not None:
            brief_text = "".join(t.strip()
                                 for para in brief_description.findall("para")
                                 for t in para.itertext()
                                 if t.strip()).strip()
            if brief_text:
                docstring += brief_text
                has_content = True
                self.print_if_verbose(f"    Brief: {brief_text}")

        # Add the detailed description. This includes the parameter list and the return value.
        if detailed_description is not None:
            detailed_content = ""
            # Add non-parameter detailed description paragraphs
            has_detailed_para = False
            for element in list(detailed_description):
                # Check if it's a para AND doesn't contain a parameterlist or simplesect (handle those separately)
                if element.tag == "para" and not element.findall(
                        ".//parameterlist") and not element.findall(
                            ".//simplesect[@kind='return']"
                        ):  # Check simplesect kind
                    para_text = "".join(t for t in element.itertext()
                                        if t.strip()).strip()
                    if para_text:
                        detailed_content += para_text + " "
                        has_detailed_para = True

            if has_detailed_para:
                if has_content:  # Add newline if brief description was present
                    docstring += "\n\n"  # Use two newlines for separation like Sphinx
                detailed_content = detailed_content.strip()
                docstring += detailed_content
                has_content = True
                self.print_if_verbose(
                    f"    Detailed Paras: {detailed_content}")

            # Add parameter docs
            parameter_list = detailed_description.find(
                ".//parameterlist[@kind='param']")  # Ensure it's for 'param'
            param_docs = ""
            if parameter_list is not None:
                self.print_if_verbose(
                    f"    Processing parameters (ignoring: {ignored_params})..."
                )
                for i, parameter_item in enumerate(
                        parameter_list.findall("./parameteritem")):
                    name_elem = parameter_item.find(
                        "./parameternamelist/parametername")
                    desc_elem = parameter_item.find(  # Description might be complex
                        "./parameterdescription/para")
                    name = name_elem.text.strip(
                    ) if name_elem is not None and name_elem.text else f'[Param {i+1}]'
                    # Handle potentially complex descriptions (multiple paras, refs, etc.)
                    desc = "".join(t for t in desc_elem.itertext() if t.strip(
                    )).strip(
                    ) if desc_elem is not None else 'No description provided'

                    if name not in ignored_params:
                        param_docs += f"{name}: {desc}\n"
                        self.print_if_verbose(f"      Param: {name} - {desc}")
                    else:
                        self.print_if_verbose(
                            f"      Ignoring documented param: {name}")

            if param_docs:
                if has_content:
                    docstring += "\n\nArgs:\n"  # Sphinx style
                else:
                    docstring += "Args:\n"
                # Strip trailing newline from last param
                docstring += param_docs.strip()
                has_content = True

            # Add return value docs
            # Doxygen uses simplesect[@kind='return'], potentially nested. Use .//
            return_sect = detailed_description.find(
                ".//simplesect[@kind='return']")
            return_doc = ""
            if return_sect is not None:
                # Find the paragraph text *within* the return section
                return_para = return_sect.find(
                    ".//para")  # Use .// just in case it's nested further
                if return_para is not None:
                    return_text = "".join(t for t in return_para.itertext()
                                          if t.strip()).strip()
                    if return_text:
                        return_doc = f"Returns: {return_text}"
                        self.print_if_verbose(f"    Return: {return_text}")

            if return_doc:
                # Decide where to put the newline
                if param_docs:  # If args were present, add newline after them
                    docstring += "\n\n" + return_doc
                elif has_content:  # If brief/detailed paras were present but no args
                    # Potential for different formatting
                    docstring += "\n\n" + return_doc
                else:  # Only return doc is present
                    docstring += return_doc
                has_content = True

        final_docstring = docstring.strip()
        if not final_docstring and not has_content:
            self.print_if_verbose(
                "    No documentation content found in brief/detailed descriptions."
            )
            return ""  # Return empty if no actual documentation was found

        return final_docstring

    def print_if_verbose(self, text: str):
        """
        Print text if the parser is in verbose mode.
        """
        if self._verbose:
            print(text)


if __name__ == "__main__":
    if len(sys.argv) < 4 or len(sys.argv) > 5:
        print(
            "Usage: python xml_parser.py <doxygen_xml_folder> <cpp_class> <cpp_method> <method_args_names (comma-separated)>"
        )
        sys.exit(1)

    parser = XMLDocParser()
    parser._verbose = True
    xml_folder = sys.argv[1]
    cpp_class = sys.argv[2]
    cpp_method = sys.argv[3]
    method_args = []
    if len(sys.argv) == 5 and sys.argv[4]:
        method_args = sys.argv[4].split(",")

    extracted_doc = parser.extract_docstring(xml_folder, cpp_class, cpp_method,
                                             method_args)

    print("\n--- Extracted Docstring ---")
    print(extracted_doc if extracted_doc else "[No documentation found]")
    print("---------------------------")
