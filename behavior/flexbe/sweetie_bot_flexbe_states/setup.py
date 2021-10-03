#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['sweetie_bot_flexbe_states', 'sweetie_bot_flexbe_states.proto2', 'sweetie_bot_flexbe_states.proxy', 'sweetie_bot_flexbe_states.internal'],
    package_dir = {'': 'src'}
)

setup(**d)
