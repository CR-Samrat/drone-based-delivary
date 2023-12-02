# Steps to run the simulator 
Step - 1: Git clone https://github.com/CR-Samrat/drone-based-delivary.git
        or download the zip file
        
Step - 2: To install BlueSky's dependencies, run the following from the BlueSky root folder:
        pip install -r requirements-gui.txt

Step - 3: Install python version >3.7 (if don't have already installed)

Step - 4: Run BlueSky.py (to check if it's working)

Step - 5: Ctrl + x "drone.py" file and paste it in the bluesky > plugins folder

Step - 6: Open settings.cfg in the root directory -> And add "drone" in the enabled_plugins array.
        enabled_plugins = ['area','datafeed','drone']

Step - 7: Run BlueSky.py

Step - 8: In the command line write the following commands 
        plugin enable drone (Enter)
        FIND_DP (Enter)
        FIND_ROUTE (Enter)
        START_DELIVARY (Enter)
