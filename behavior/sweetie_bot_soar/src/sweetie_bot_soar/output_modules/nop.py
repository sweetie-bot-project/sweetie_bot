from . import output_module

class NOp(output_module.OutputModule):

    def __init__(self, config):
        super(NOp, self).__init__("nop")

output_module.register("nop", NOp)
