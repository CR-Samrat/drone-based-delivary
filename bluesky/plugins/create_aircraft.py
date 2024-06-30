""" My plugin for creating aircraft in BlueSky. """

from bluesky import core, stack, traf

class CreateAircraft(core.Entity):
    """ This plugin creates new aircraft. """
    def __init__(self):
        super().__init__()

    @stack.command
    def create(self, acid: str, type: str, callsign: str = 'UNKN'):
        """ Create a new aircraft. """
        with self.settrafarrays():
            if acid in traf.id:
                return False, f'Aircraft with ACID {acid} already exists.'
            traf.id.append(acid)
            traf.type.append(type)
            traf.callsign.append(callsign)

            # Initialize other aircraft states as needed
            # ...

        return True, f'Aircraft {callsign} created.'
