#!/usr/bin/env python3
"""
Helper script to wrap C++ to Python with Pybind.
This script is installed via CMake to the user's binary directory
and invoked during the wrapping by CMake.
"""

# pylint: disable=import-error

import argparse

from gtwrap.pybind_wrapper import PybindWrapper


def main():
    """Main runner."""
    arg_parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    arg_parser.add_argument("--src",
                            type=str,
                            required=True,
                            help="Input interface .i/.h file")
    arg_parser.add_argument(
        "--module_name",
        type=str,
        required=True,
        help="Name of the Python module to be generated and "
        "used in the Python `import` statement.",
    )
    arg_parser.add_argument(
        "--out",
        type=str,
        required=True,
        help="Name of the output pybind .cc file",
    )
    arg_parser.add_argument(
        "--use-boost",
        action="store_true",
        help="using boost's shared_ptr instead of std's",
    )
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
        "and `from <module_name> import ns4` gives you access to a Python "
        "`ns4.Class` of the C++ `ns1::ns2::ns3::ns4::Class`. ",
    )
    arg_parser.add_argument(
        "--ignore",
        nargs='*',
        type=str,
        help="A space-separated list of classes to ignore. "
        "Class names must include their full namespaces.",
    )
    arg_parser.add_argument("--template",
                            type=str,
                            help="The module template file")
    args = arg_parser.parse_args()

    top_module_namespaces = args.top_module_namespaces.split("::")
    if top_module_namespaces[0]:
        top_module_namespaces = [''] + top_module_namespaces

    with open(args.template, "r") as f:
        template_content = f.read()

    wrapper = PybindWrapper(
        module_name=args.module_name,
        use_boost=args.use_boost,
        top_module_namespaces=top_module_namespaces,
        ignore_classes=args.ignore,
        module_template=template_content,
    )

    # Wrap the code and get back the cpp/cc code.
    sources = args.src.split(';')
    wrapper.wrap(sources, args.out)


if __name__ == "__main__":
    main()
