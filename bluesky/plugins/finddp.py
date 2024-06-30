from bluesky import core, stack, traf
import numpy as np
from random import randint

def init_plugin():

    finddp = Finddp()

    config = {
        'plugin_name':'FINDDP',
        'plugin_type':'sim',
    }

    return config

class Finddp(core.Entity):
    def __init__(self):
        super().__init__()

    @stack.command
    def testing(self, dp: str):
        return True, f'Working {dp}'