from setuptools import setup, find_packages

setup(
    name="module6",  # name of your package
    version="0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
)
