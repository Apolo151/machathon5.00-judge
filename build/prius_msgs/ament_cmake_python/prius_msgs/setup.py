from setuptools import find_packages
from setuptools import setup

setup(
    name='prius_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('prius_msgs', 'prius_msgs.*')),
)
