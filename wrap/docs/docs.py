import xml.etree.ElementTree as ET


class Doc():
    def __init__(self, tree):
        self.tree = tree

    def get_tree(self):
        """Get this Doc's tree.

        Returns:
            The xml.etree.ElementTree object of the documentation.
        """
        return self.tree

    def __eq__(self, other):
        if other is None or other.get_tree() is None:
            return None

        return ET.tostring(self.tree.getroot()) == \
            ET.tostring(other.get_tree().getroot())


class ClassDoc(Doc):
    pass


class FreeDoc(Doc):
    pass


class Docs():
    def __init__(self, class_docs, free_docs):
        # These are dicts that map file_path -> Doc
        self.class_docs = class_docs
        self.free_docs = free_docs

    def get_class_docs(self, class_name):
        '''Get the documentation for the class.

        Arguments:
        class_name -- the name of the class

        Returns:
        The ClassDoc with the class's documentation. None if the class does not
        exist.
        '''
        return self.class_docs.get(class_name)

    def get_free_docs(self, free_func_name):
        '''Get the documentation for a free function.

        Arguments:
        free_func_name -- the name of the free function

        Returns:
        The FreeDoc with the free function's documentation. None if the class
        does not exist.
        '''
        return self.free_docs.get(free_func_name)

    def get_class_docs_keys_list(self):
        return list(self.class_docs)

    def get_free_docs_keys_list(self):
        return list(self.free_docs)

    def get_class_docs_values_list(self):
        return list(self.class_docs.values())

    def get_free_docs_values_list(self):
        return list(self.free_docs.values())
