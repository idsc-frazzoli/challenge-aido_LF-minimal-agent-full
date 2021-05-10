from setuptools import setup


def get_version(filename):
    import ast

    version = None
    with open(filename) as f:
        for line in f:
            if line.startswith("__version__"):
                version = ast.parse(line).body[0].value.s
                break
        else:
            raise ValueError("No version found in %r." % filename)
    if version is None:
        raise ValueError(filename)
    return version


install_requires = [
    "duckietown-world-daffy",
    "aido-agents-daffy"
]

module = "gtduckie"
package = "gtduckie"
src = "src"

version = get_version(filename=f"src/{module}/__init__.py")

setup(
    name=package,
    package_dir={"": src},
    packages=[module],
    version=version,
    zip_safe=False,
    install_requires=install_requires,
)
