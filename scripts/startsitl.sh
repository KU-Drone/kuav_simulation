#!/bin/bash

#for directly running the script
# SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# cd ../ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
# cd ../ardupilot/ArduCopter

if [ "$#" -ge "1" ]; then
    DIR=$1
else
    DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
fi

if [[ "$(basename "$DIR")" != "ArduPlane" ]]; then
    echo "Not in ArduPlane folder, stopping execution"
    exit 1
fi

echo "$DIR"
cd $DIR
pwd
sim_vehicle.py -v ArduPlane -f gazebo-zephyr  -m --mav10 --map --console -I0
