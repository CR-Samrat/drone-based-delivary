from random import randint
import numpy as np
from bluesky import core, stack, traf
import math

def init_plugin():
    drone = Adv_drone()

    config = {
        'plugin_name':     'MULTI_CAPACITY_DELIVERY',
        'plugin_type':     'sim',
        }

    return config

class Adv_drone(core.Entity):
    def __init__(self):
        super().__init__()

        stack.stack(f'BOX Eucledian 22.5726 88.4010 22.6141 88.4354')
        stack.stack(f'BOX Manhattan 22.5726 88.4354 22.6141 88.4654')

    # create new drone
    @stack.command
    def create_drone(self, acid: str, lat:float, lon:float):
        stack.stack(f'CRE {acid} A225 {lat} {lon} 130 FL410 485')
