from setuptools import find_packages
from setuptools import setup

setup(
    name='car_demo',
    version='0.0.1',
    packages=find_packages(
        include=('car_demo', 'car_demo.*')),
)
