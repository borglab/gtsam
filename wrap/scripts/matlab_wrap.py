#!/usr/bin/env python3

"""
Helper script to wrap C++ to Matlab.
This script is installed via CMake to the user's binary directory
and invoked during the wrapping by CMake.
"""

import argparse
import os

import gtwrap.interface_parser as parser
import gtwrap.template_instantiator as instantiator
from gtwrap.matlab_wrapper import MatlabWrapper, generate_content

if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    arg_parser.add_argument("--src", type=str, required=True,
                            help="Input interface .h file.")
    arg_parser.add_argument("--module_name", type=str, required=True,
                            help="Name of the C++ class being wrapped.")
    arg_parser.add_argument("--out", type=str, required=True,
                            help="Name of the output folder.")
    arg_parser.add_argument(
        "--top_module_namespaces",
        type=str,
        default="",
        help="C++ namespace for the top module, e.g. `ns1::ns2::ns3`. "
        "Only the content within this namespace and its sub-namespaces "
        "will be wrapped. The content of this namespace will be available at "
        "the top module level, and its sub-namespaces' in the submodules.\n"
        "For example, `import <module_name>` gives you access to a Python "
        "`<module_name>.Class` of the corresponding C++ `ns1::ns2::ns3::Class`"
        ", and `from <module_name> import ns4` gives you access to a Python "
        "`ns4.Class` of the C++ `ns1::ns2::ns3::ns4::Class`. ")
    arg_parser.add_argument("--ignore",
                            nargs='*',
                            type=str,
                            help="A space-separated list of classes to ignore. "
                                 "Class names must include their full namespaces.")
    args = arg_parser.parse_args()

    top_module_namespaces = args.top_module_namespaces.split("::")
    if top_module_namespaces[0]:
        top_module_namespaces = [''] + top_module_namespaces

    with open(args.src, 'r') as f:
        content = f.read()

    if not os.path.exists(args.src):
        os.mkdir(args.src)

    module = parser.Module.parseString(content)

    instantiator.instantiate_namespace_inplace(module)

    import sys

    print("Ignoring classes: {}".format(args.ignore), file=sys.stderr)
    wrapper = MatlabWrapper(module=module,
                            module_name=args.module_name,
                            top_module_namespace=top_module_namespaces,
                            ignore_classes=args.ignore)

    cc_content = wrapper.wrap()

    generate_content(cc_content, args.out)
