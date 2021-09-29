#!/usr/bin/env python2

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_quadrotor_safety'],
    package_dir={'': 'src'},
)

setup(**d)
