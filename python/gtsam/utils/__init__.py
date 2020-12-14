import glob
import os.path as osp


def findExampleDataFile(name):
    """
    Find the example data file specified by `name`.
    """
    # This is okay since gtsam will already be loaded
    # before this function is accessed. Thus no effect.
    import gtsam

    site_package_path = gtsam.__path__[0]
    # add the * at the end to glob the entire directory
    data_path = osp.join(site_package_path, "Data", "*")

    extensions = ("", ".graph", ".txt", ".out", ".xml", ".g2o")

    for data_file_path in glob.glob(data_path, recursive=True):
        for ext in extensions:
            if (name + ext) == osp.basename(data_file_path):
                return data_file_path
