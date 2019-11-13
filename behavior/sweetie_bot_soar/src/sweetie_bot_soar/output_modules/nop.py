import output_module
from output_module import OutputModule

class NOp(OutputModule):

    def __init__(self, config):
        super(NOp, self).__init__("nop")

output_module.register("nop", NOp)
