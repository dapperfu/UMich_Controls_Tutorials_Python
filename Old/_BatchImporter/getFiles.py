import os
# Use the built-in version of scandir/walk if possible,
# otherwise use the os module version
try:
    # https://github.com/benhoyt/scandir
    # Accepted into Python 3.5 stdlib
    # Increases speed 2-20 times (depending on the platform and file system)
    from scandir import walk
except ImportError:
    # The old way with OS.
    from os import walk

def getDirs(
        directory=os.path.curdir,
        depth=None,
        verbose=False,
        absolute=True,
        ignore=None):
    currDepth = 0
    folderList = list()

    if absolute:
        directory = os.path.abspath(directory)

    for root, dirs, files in walk(directory, topdown=True):
        currDepth += 1
        for dir in dirs:
            if dir == ".zfs":
                continue
            folderList.append(os.path.join(root,dir))
        if depth is None:
            continue
        if currDepth >= depth:
            break
    return folderList

def getFiles(
        directory=os.path.curdir,
        extensions=None,
        depth=None,
        verbose=False,
        absolute=True,
        ignore=None):
    """
    Args:
        directory (str):   Directory to scan.
        extensions (list): Extensions to scan for
        depth (int):       Depth to recurse into the directories
        absolute (bool):   Return the absolute path, defaults to True.
        ignore ():

    Returns:
        list: List of all files found in ``directory`` with extension in ``ext``
              to depth 

    Raises:
    """
    currDepth = 0
    fileList = list()

    if isinstance(extensions, str):
        extensions = {extensions}

    # To lower case.
    extensions=[extension.lower() for extension in extensions]
    
    
    if absolute:
        directory = os.path.abspath(directory)

    for root, dirs, files in walk(directory, topdown=True):
        currDepth += 1
        for name in files:
            file = os.path.join(root, name)
            ext = os.path.splitext(file)[1].lower()
            if extensions:
                if ext in extensions:
                    fileList.append(file)
            else:
                fileList.append(file)
        if depth is None:
            continue
        if currDepth >= depth:
            break
    return fileList

def getFilesGen(
        directory=os.path.curdir,
        extensions=None,
        depth=None,
        verbose=False,
        absolute=True,
        ignore=None):
    """
    Args:
        directory (str):   Directory to scan.
        extensions (list): Extensions to scan for
        depth (int):       Depth to recurse into the directories
        absolute (bool):   Return the absolute path, defaults to True.
        ignore ():

    Returns:
        list: List of all files found in ``directory`` with extension in ``ext``
              to depth 

    Raises:
    """
    currDepth = 0
    fileList = list()

    if isinstance(extensions, str):
        extensions = {extensions}

    # To lower case.
    extensions=[extension.lower() for extension in extensions]
    
    if absolute:
        directory = os.path.abspath(directory)

    for root, dirs, files in walk(directory, topdown=True):
        currDepth += 1
        for name in files:
            file = os.path.join(root, name)
            ext = os.path.splitext(file)[1].lower()
            if extensions:
                if ext in extensions:
                    yield file
            else:
                yield file
        if depth is None:
            continue
        if currDepth >= depth:
            break
