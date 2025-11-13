from setuptools import find_packages
from setuptools import setup

setup(
    name='mecanum_robot',
    version='1.0.0',
    packages=find_packages(
        include=('mecanum_robot', 'mecanum_robot.*')),
)
