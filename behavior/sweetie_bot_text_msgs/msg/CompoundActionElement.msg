# The part of compound action.
#
#  * seq_num --- the number of previous TextActionSpec after which this action should be executed.
#  * seq_delay --- delay between previous action and current action (seconds).
#  * cmd --- text command to be executed.
#  
int32 seq_num
float64 seq_delay
sweetie_bot_text_msgs/TextCommand cmd
