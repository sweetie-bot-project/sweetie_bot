import output_module
from output_module import OutputModule

class Nop(OutputModule):

    def __init__(self, config):
        super(Nop, self).__init__("nop")

output_module.register("nop", Nop)
