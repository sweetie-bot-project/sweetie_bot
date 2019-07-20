# Compound action specified by sequence of TextActions.
#
# Actions specified by CompoundActionPart are executed one after another or simultaneously.
# CompoundActionPart.seq_num defines the number of previous action (numeration starts from zero). 
# If seq_num is equal to zero the action is executed first. CompoundActionPart.seq_delay specifies 
# delay after the end of the previous action. Action itself is defined by TextCommand CompoundActionPart.cmd.
#  
sweetie_bot_text_msgs/CompoundActionElement[] action_list
