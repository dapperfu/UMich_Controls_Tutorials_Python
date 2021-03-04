import os

import setuptools

import versioneer

LOCAL_DIR = os.path.dirname(os.path.abspath(__file__))


def read_requirements(path="requirements.txt"):
    """Read requirements file relative to setup.py"""
    full_path = os.path.join(LOCAL_DIR, path)
    if not os.path.exists(full_path):
        return []

    def yield_line(path):
        with open(path) as fid:
            yield from fid.readlines()

    return [
        requirement.strip()
        for requirement in yield_line(full_path)
        if not requirement.startswith("#")
    ]


requirements = read_requirements()
print(requirements)
# test_requirements = read_requirements(path="requirements_test.txt")

setuptools.setup(
    version=versioneer.get_version(),
    cmdclass=versioneer.get_cmdclass(),
    name="UMich Controls - Python",
    description="GCode",
    license="BSD",
    packages=setuptools.find_packages(),
    zip_safe=False,
    install_requires=[],
    include_package_data=True,
)
