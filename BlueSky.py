""" Main BlueSky start script """
import sys
from bluesky.__main__ import main
from bluesky import traf


if __name__ == "__main__":
    # traf.cre("WB10", "A225", 20.171466, 77.113148, 130, "FL410", 485)
    sys.exit(main())
