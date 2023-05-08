# Command identified by text string.
#
#  * type --- type of action. 
#  * command --- text action identificator. A
#  * args --- optional arguments names or list of arguments.
#  * values --- optianal arguments values.
#
#  Usage of args is not recommended. Text indentifer sould have clear and definite semantic without any arguments.
#  
#  Example: { type : eyes/emotion, command: happy }
#  Example: { type : joint_trajectory, command: greeting }
#  
string type
string command
string options
#strings args[]
#float64 values[]
