"""Code to help instantiate templated classes, methods and functions."""

# pylint: disable=too-many-arguments, too-many-instance-attributes, no-self-use, no-else-return, too-many-arguments, unused-format-string-argument, unused-variable. unused-argument, too-many-branches

from typing import Iterable, Sequence, Union

import gtwrap.interface_parser as parser
from gtwrap.template_instantiator.classes import *
from gtwrap.template_instantiator.constructor import *
from gtwrap.template_instantiator.declaration import *
from gtwrap.template_instantiator.function import *
from gtwrap.template_instantiator.helpers import *
from gtwrap.template_instantiator.method import *
from gtwrap.template_instantiator.namespace import *
