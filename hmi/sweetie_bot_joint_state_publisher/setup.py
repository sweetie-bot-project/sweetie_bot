from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sweetie_bot_joint_state_publisher'],
    package_dir={'': 'src'}
)

setup(**d)
