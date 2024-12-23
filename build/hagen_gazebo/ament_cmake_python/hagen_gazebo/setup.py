from setuptools import find_packages
from setuptools import setup

setup(
    name='hagen_gazebo',
    version='1.0.0',
    packages=find_packages(
        include=('hagen_gazebo', 'hagen_gazebo.*')),
)
