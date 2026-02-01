"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Various common utilities.

Author: Varun Agrawal
"""


def collect_namespaces(obj):
    """
    Get the chain of namespaces from the lowest to highest for the given object.

    Args:
        obj: Object of type Namespace, Class, InstantiatedClass, or Enum.
    """
    namespaces = []
    ancestor = obj.parent
    while ancestor and ancestor.name:
        namespaces = [ancestor.name] + namespaces
        ancestor = ancestor.parent
    return [''] + namespaces
