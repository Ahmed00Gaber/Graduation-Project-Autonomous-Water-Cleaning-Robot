#!/bin/bash

# Set display environment variables
export DISPLAY=:0.0
source venvyolov5/bin/activate
# Change to the directory containing your Python files
cd /home/pi/yolov5/grad

# Run the server scripts in the background
nohup bash -c "python3 server_ada.py" > step1_output.txt 2> step1_error.txt &


# Sleep for a while to allow the server scripts to start


sleep 10

# Run the client scripts in the background
nohup bash -c "python3 client_tds.py" > step3_output.txt 2> step3_error.txt &
nohup bash -c "python3 client_ph.py" > step4_output.txt 2> step4_error.txt &
nohup bash -c "python3 client_gps.py" > step5_output.txt 2> step5_error.txt &
