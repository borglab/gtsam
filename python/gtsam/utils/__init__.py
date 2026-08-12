from pathlib import Path


def findExampleDataFile(name):
    """
    Find the example data file specified by `name`.
    """
    # This is okay since gtsam will already be loaded
    # before this function is accessed. Thus no effect.
    import gtsam

    package_path = Path(gtsam.__path__[0]).resolve()
    requested = Path(name)
    roots_to_search = []

    for parent in package_path.parents:
        for candidate in (
            parent / "examples" / "Data",
            parent / "gtsam_examples" / "Data",
        ):
            if candidate.exists():
                roots_to_search.append(candidate)

    package_data = package_path / "Data"
    if package_data.exists():
        roots_to_search.append(package_data)

    unique_roots = []
    for root in roots_to_search:
        if root not in unique_roots:
            unique_roots.append(root)

    extensions = ("", ".graph", ".txt", ".out", ".xml", ".g2o")
    for root in unique_roots:
        for ext in extensions:
            candidate = root / (name + ext)
            if candidate.is_file() or candidate.is_dir():
                return str(candidate)

    if len(requested.parts) == 1:
        for root in unique_roots:
            for candidate in root.iterdir():
                for ext in extensions:
                    if candidate.name == name + ext:
                        return str(candidate)

    raise FileNotFoundError(
        f"Could not find example data file '{name}' under any known GTSAM example data root."
    )
