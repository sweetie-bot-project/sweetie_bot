cmake_minimum_required(VERSION 3.0.2)
project(sweetie_bot_ai_core)

find_package(catkin REQUIRED)

## Pure-python package: expose the module on PYTHONPATH via setup.py.
catkin_python_setup()

catkin_package()

#############
## Testing ##
#############
# Run python tests with: catkin_add_nosetests(tests) or pytest directly against src/.
