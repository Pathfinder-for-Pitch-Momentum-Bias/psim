
# psim Python Interface

This folder will house all python code in this repository. Functionality to be
supported by Python may include, but is not limited to:

 * Running a simulation in real time with one or two satellites.
 * Real time graphing utilities and other data visulation tools to be used in
   conjunction with a real time simulation. To use this utility run

       python -m usb_console.plotter -d /path/to/data/file

   after running the installation steps below. The `/path/to/data/file` needs to be
   produced by the simulation.

 * Hardware/Sim interface for SHITLs.

# Installing

To install the Python MATLAB engine, you must:

 1. Install Python 3.6 - it's what we'll be using as it's the newest version
    officially supported by MATLAB.
 2. Create and activate a virtualenv within this directory.
    - For Mac and Linux: `python3 -m virtualenv venv; source venv/bin/activate`
    - For Windows: `python -m virtualenv venv`, followed by `venv\Scripts\activate`.
 3. Install the requirements: `pip install -r requirements.txt`
 4. If on Mac, separately install `readline` and `pty` via `pip`.
 5. I found that I required MATLAB R2019b in order for the simulation to work. Make sure you
    have this version.
 6. Set MATLAB's `pyversion` variable to link with the newly installed Python 3.6.
    You set and check the Python interpretter used by MATLAB by entering
    something like the following in MATLAB's terminal. Replace `PATH_TO_PSIM` with 
    the absolute path to this repository. For windows, use `PATH_TO_PSIM/python/venv/Scripts/python`.

        >> pyversion PATH_TO_PSIM/python/venv/bin/python3
        >> pyversion

            version: '3.6'
            executable: '/Users/tanishqaggarwal/Documents/pan/repositories/psim/python/venv/bin/python3'
            library: ''
            home: '/Users/tanishqaggarwal/Documents/pan/repositories/psim/python/venv/bin/..'
            isloaded: 0

 7. Install MATLAB for this repository. On macOS:

         cd /Applications/MATLAB_R2019b.app/extern/engines/python
         python setup.py install --prefix="PATH_TO_PSIM/python/venv"

      On Windows:

         cd C:\Program Files\MATLAB\R2019b\extern\engines\python
         python setup.py install --prefix="PATH_TO_PSIM/python/venv"


# Configuring Simulation

**Running with Teensy in the loop**

Open the config file:

    psim/python/usb_console/configs

Change the port to the COM port Teensy is connected to

Run the main script:

    python -m usb_console.run_simulation -c usb_console/configs/fc_only_teensy.json

**Running with software only**

This only works for Mac and Linux for now.

Edit `usb_console/configs/fc_only.json` so that the `binary_filepath` for the Flight Controller points to a binary built
for the Flight Software. You can find these binaries [here](https://github.com/pathfinder-for-autonomous-navigation/FlightSoftware/releases).

# Running simulation
1. Activate the virtualenv as done in step 2 of the Installation above. You do not need to reinstall the venv.
    - For Mac and Linux: `source venv/bin/activate`
    - For Windows: `venv\Scripts\activate`

2. Then, run the main script. On Windows you must run

       python -m usb_console.run_simulation -c usb_console/configs/fc_only_teensy.json

      since psim for Windows currently only supports connections to actual Teensy devices. On Mac you may run

       python -m usb_console.run_simulation -c usb_console/configs/fc_only_native.json

      To connect to a desktop binary.

3. If you'd like to run more complex configurations, see the examples in the folder `usb_console/configs`.