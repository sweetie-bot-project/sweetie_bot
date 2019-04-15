from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
        packages = ['sweetie_bot_clop_generator'],
        package_dir = {'': 'pysrc'}
    )

setup(**setup_args)
