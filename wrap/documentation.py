import os.path as path
import docs.parser.parse_xml as parser


if __name__ == "__main__":
    # TODO: Choose file in terminal
    parser.generate_xml(path.realpath('tests/NonlinearFactor.h'))
    parser.parse()
